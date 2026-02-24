"""
AImarketdb.py - MySQL 초기화 스크립트
fastapi_db 데이터베이스에 orders, order_items 테이블을 생성합니다.
AImarket.py와 별도로 실행하는 DB 초기화 유틸리티입니다.

[사용법]
  python3 AImarketdb.py

[환경변수]
  DB_HOST     : MySQL 호스트 (기본값: 127.0.0.1)
  DB_PORT     : MySQL 포트 (기본값: 3306)
  DB_USER     : MySQL 사용자 (기본값: root)
  DB_PASSWORD : MySQL 비밀번호
  DB_NAME     : 데이터베이스명 (기본값: fastapi_db)
"""
import pymysql
import os

DB_CONFIG = {
    'host':       os.environ.get('DB_HOST', '127.0.0.1'),
    'port':       int(os.environ.get('DB_PORT', 3306)),
    'user':       os.environ.get('DB_USER', 'root'),
    'password':   os.environ.get('DB_PASSWORD', ''),
    'db':         os.environ.get('DB_NAME', 'fastapi_db'),
    'charset':    'utf8mb4',
    'cursorclass': pymysql.cursors.DictCursor
}

conn = pymysql.connect(**DB_CONFIG)
cursor = conn.cursor()

try:
    # 주문 테이블
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS orders (
        id INT AUTO_INCREMENT PRIMARY KEY,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        total_price INT,
        status VARCHAR(50)
    )
    """)

    # 주문 상세 테이블
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS order_items (
        id INT AUTO_INCREMENT PRIMARY KEY,
        order_id INT,
        product VARCHAR(100),
        quantity INT,
        price INT,
        item_total INT,
        FOREIGN KEY(order_id) REFERENCES orders(id)
    )
    """)

    conn.commit()
    print("fastapi_db 테이블(orders, order_items) 생성/확인 완료")

except Exception as e:
    print(f"Error: {e}")
    conn.rollback()
finally:
    conn.close()
