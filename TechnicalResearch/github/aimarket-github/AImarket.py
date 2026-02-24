import sys
import re
import pymysql
import hashlib
import urllib.request
import urllib.error
import os
import threading
import subprocess
from datetime import datetime
from dataclasses import dataclass

# .env 파일 로드 (python-dotenv 설치 필요: pip install python-dotenv)
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # python-dotenv 미설치 시 환경변수를 직접 설정하세요
from PyQt6.QtCore import Qt, QSize, pyqtSignal, QTimer, QRegularExpression
from PyQt6.QtGui import QPixmap, QPainter, QFont, QColor, QAction, QRegularExpressionValidator
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QSpinBox,
    QListWidget, QListWidgetItem, QMessageBox,
    QFrame, QScrollArea, QSizePolicy
)

# --- ROS 2 Integration ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.context import Context
    from std_msgs.msg import Int32MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️ rclpy not found. ROS 2 communication will be disabled.")


class ROS2DomainManager:
    """Manages ROS 2 communication for a specific Domain ID"""
    def __init__(self, domain_id, robot_name, topic_name='/pick_items'):
        self.domain_id = domain_id
        self.robot_name = robot_name
        self.context = Context()
        self.context.init(domain_id=domain_id)
        self.node = Node(f'market_client_{robot_name}', context=self.context)
        self.publisher = self.node.create_publisher(Int32MultiArray, topic_name, 10)
        
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()
        print(f"🚀 ROS 2 Manager for {robot_name} started on Domain {domain_id} (Topic: {topic_name})")

    def spin(self):
        try:
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            executor.add_node(self.node)
            executor.spin()
        except Exception as e:
            print(f"ROS 2 Spin Error ({self.robot_name}): {e}")
        finally:
            self.node.destroy_node()
            if self.context.ok():
                self.context.shutdown()

    def publish_pick(self, marker_ids):
        msg = Int32MultiArray()
        msg.data = [int(mid) for mid in marker_ids]
        self.publisher.publish(msg)
        print(f"📡 [Domain {self.domain_id}] Published markers: {marker_ids}")

# Initialize Global Managers if available
ros2_arm17 = None
ros2_arm18 = None

if ROS2_AVAILABLE:
    try:
        # Arm 17: Domain 7, Topic /pick_items
        ros2_arm17 = ROS2DomainManager(7, "arm17", "/pick_items")
        # Arm 18: Domain 19, Topic /robot_arm_18/pick_items
        ros2_arm18 = ROS2DomainManager(19, "arm18", "/robot_arm_18/pick_items")
    except Exception as e:
        print(f"❌ Failed to initialize ROS 2: {e}")
        ROS2_AVAILABLE = False



# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
# .env 파일 또는 환경변수에서 읽어옵니다.
# .env 파일 예시:
#   API_SERVER_URL=http://your-server-ip:8000
#   DB_HOST=127.0.0.1
#   DB_PORT=3306
#   DB_USER=root
#   DB_PASSWORD=your_password
#   DB_NAME=fastapi_db
API_SERVER_URL = os.environ.get("API_SERVER_URL", "http://your-server-ip:8000")

# ---------------------------------------------------------
# Database Configuration (MySQL)
# ---------------------------------------------------------
DB_CONFIG = {
    'host': os.environ.get('DB_HOST', '127.0.0.1'),
    'port': int(os.environ.get('DB_PORT', 3306)),
    'user': os.environ.get('DB_USER', 'root'),
    'password': os.environ.get('DB_PASSWORD', ''),
    'db': os.environ.get('DB_NAME', 'fastapi_db'),
    'charset': 'utf8mb4',
    'cursorclass': pymysql.cursors.DictCursor
}

def get_db_connection():
    try:
        conn = pymysql.connect(**DB_CONFIG)
        return conn
    except pymysql.MySQLError as e:
        print(f"Error connecting to MySQL: {e}")
        raise e

def init_db_and_migrate():
    """DB 연결 확인, 테이블 생성 및 스키마 마이그레이션"""
    print(f"Connecting to MySQL database: {DB_CONFIG['db']}")
    
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # 1. 테이블 생성 (MySQL Syntax)
        # Users
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id INT AUTO_INCREMENT PRIMARY KEY,
                name VARCHAR(100) NOT NULL,
                age INT NOT NULL,
                address VARCHAR(200) NOT NULL,
                created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
                updated_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
                login_id VARCHAR(50) UNIQUE,
                password VARCHAR(200),
                birth_date VARCHAR(8)
            )
        """)
        
        # Products
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS products (
                id INT AUTO_INCREMENT PRIMARY KEY,
                name VARCHAR(100) NOT NULL,
                category VARCHAR(50) NOT NULL,
                price INT NOT NULL DEFAULT 0,
                stock INT NOT NULL DEFAULT 0,
                description TEXT,
                marker_id INT UNIQUE
            )
        """)
        
        # Orders
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS orders (
                id INT AUTO_INCREMENT PRIMARY KEY,
                user_id INT NOT NULL,
                total_price INT NOT NULL DEFAULT 0,
                status VARCHAR(50) NOT NULL,
                created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY(user_id) REFERENCES users(id)
            )
        """)
        
        # Order Items
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS order_items (
                id INT AUTO_INCREMENT PRIMARY KEY,
                order_id INT NOT NULL,
                product_id INT NOT NULL,
                quantity INT NOT NULL,
                price_at_order INT NOT NULL,
                FOREIGN KEY(order_id) REFERENCES orders(id),
                FOREIGN KEY(product_id) REFERENCES products(id)
            )
        """)
        
        # 2. 마이그레이션 (컬럼 확인)
        # MySQL에서는 information_schema.columns를 조회하거나 DESCRIBE 사용
        cursor.execute("DESCRIBE users")
        columns = [row['Field'] for row in cursor.fetchall()]
        
        if "login_id" not in columns:
            print("Migrating: Adding login_id to users...")
            cursor.execute("ALTER TABLE users ADD COLUMN login_id VARCHAR(50) UNIQUE")
        
        if "password" not in columns:
            print("Migrating: Adding password to users...")
            cursor.execute("ALTER TABLE users ADD COLUMN password VARCHAR(200)")
            
        if "birth_date" not in columns:
            print("Migrating: Adding birth_date to users...")
            cursor.execute("ALTER TABLE users ADD COLUMN birth_date VARCHAR(8)")

        # 3. 초기 상품 데이터 삽입 (데이터가 없을 경우에만)
        cursor.execute("SELECT count(*) as cnt FROM products")
        if cursor.fetchone()['cnt'] == 0:
            print("Inserting initial products...")
            initial_products = [
                ("MacBook Air", "hat", 1300000, 10, "M2 Chip"),
                ("Magic Mouse", "hat", 89000, 50, "Wireless"),
                ("AirPods Pro", "hat", 329000, 30, "Noise Cancelling"),
                ("USB-C Cable", "hat", 25000, 100, "2m"),
                ("Studio Display", "hat", 2090000, 2, "5K Retina"),
                ("Magic Keyboard", "hat", 179000, 20, "With Touch ID"),
            ]
            cursor.executemany("""
                INSERT INTO products (name, category, price, stock, description)
                VALUES (%s, %s, %s, %s, %s)
            """, initial_products)
            
        conn.commit()
        print("Database initialization completed.")
        
    except Exception as e:
        print(f"Migration error: {e}")
        if 'conn' in locals():
            conn.rollback()
    finally:
        if 'conn' in locals():
            conn.close()

# ---------------------------------------------------------
# Stylesheet (Mac Aesthetic)
# ---------------------------------------------------------
STYLESHEET = """
QWidget {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    color: #1D1D1F;
    font-size: 14px;
}

/* Main Background */
QMainWindow, QStackedWidget {
    background-color: #F5F5F7;
}

/* Headings */
QLabel#Title {
    font-size: 28px;
    font-weight: 700;
    color: #1D1D1F;
    margin-bottom: 10px;
}
QLabel#Subtitle {
    font-size: 18px;
    font-weight: 600;
    color: #1D1D1F;
    margin-bottom: 4px;
}
QLabel#LabelGray {
    color: #86868B;
    font-size: 13px;
}

/* Cards / Containers */
QFrame#Card, QGroupBox {
    background-color: #FFFFFF;
    border: 1px solid #D2D2D7;
    border-radius: 12px;
}
QGroupBox {
    margin-top: 24px;
    padding-top: 24px;
    font-weight: 700;
    color: #1D1D1F;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
}

/* Input Fields */
QLineEdit, QSpinBox {
    background-color: #FFFFFF;
    border: 1px solid #D2D2D7;
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 14px;
    selection-background-color: #007AFF;
}
QLineEdit:focus, QSpinBox:focus {
    border: 2px solid #007AFF;
    outline: none;
}

/* Lists */
QListWidget {
    background-color: #FFFFFF;
    border: 1px solid #D2D2D7;
    border-radius: 12px;
    padding: 8px;
    outline: none;
}
QListWidget::item {
    padding: 8px;
    border-radius: 6px;
}
QListWidget::item:selected {
    background-color: #F5F5F7;
    color: #1D1D1F;
}

/* Buttons */
/* Default (Secondary) */
QPushButton {
    background-color: #FFFFFF;
    border: 1px solid #D2D2D7;
    border-radius: 8px;
    padding: 8px 16px;
    font-weight: 500;
    color: #1D1D1F;
}
QPushButton:hover {
    background-color: #F5F5F7;
    border-color: #C6C6C6;
}
QPushButton:pressed {
    background-color: #ECECEC;
}

/* Primary Action Button (Blue) */
QPushButton#Primary {
    background-color: #007AFF;
    border: 1px solid #007AFF;
    color: #FFFFFF;
    font-weight: 600;
}
QPushButton#Primary:hover {
    background-color: #0062CC;
    border-color: #0062CC;
}
QPushButton#Primary:pressed {
    background-color: #0051A8;
}

/* Navigation Button (Transparent-ish) */
QPushButton#Nav {
    background-color: transparent;
    border: none;
    color: #007AFF;
    font-weight: 500;
    text-align: left;
}
QPushButton#Nav:hover {
    text-decoration: underline;
}

/* Scroll Area */
QScrollArea {
    border: none;
    background: transparent;
}
QScrollBar:vertical {
    border: none;
    background: #F5F5F7;
    width: 10px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #C1C1C1;
    min-height: 20px;
    border-radius: 5px;
}

/* Fix for QMessageBox on macOS (Text visibility) */
QMessageBox {
    background-color: #FFFFFF;
}
QMessageBox QLabel {
    color: #1D1D1F;
}
"""

@dataclass
class Product:
    name: str
    price: int
    stock: str

def make_dummy_image(text: str, w=200, h=160) -> QPixmap:
    pix = QPixmap(w, h)
    pix.fill(QColor("#F2F2F7"))
    p = QPainter(pix)
    p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    
    # Text
    p.setPen(QColor("#6E6E73"))
    font = QFont() # Use default system font
    font.setPixelSize(14)
    font.setBold(True)
    p.setFont(font)
    p.drawText(pix.rect(), Qt.AlignmentFlag.AlignCenter, text)
    
    # Border
    p.setPen(QColor("#E5E5EA"))
    p.drawRect(0, 0, w-1, h-1)
    p.end()
    return pix

# ---------------------------------------------------------
# Components
# ---------------------------------------------------------
class Navbar(QWidget):
    def __init__(self, title, left_btns=None, right_btns=None):
        super().__init__()
        self.setFixedHeight(60)
        self.setStyleSheet("background-color: #F5F5F7; border-bottom: 1px solid #D2D2D7;")
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 0, 24, 0)
        
        # Left container
        if left_btns:
            for btn in left_btns:
                layout.addWidget(btn)
        
        layout.addStretch()
        
        # Title (Center-ish)
        lbl = QLabel(title)
        lbl.setStyleSheet("font-size: 18px; font-weight: 700; color: #1D1D1F; border: none;")
        layout.addWidget(lbl)
        
        layout.addStretch()
        
        # Right container
        if right_btns:
            for btn in right_btns:
                layout.addWidget(btn)
        
        # User ID Label (Redesigned: Mac-style Pill)
        self.user_lbl = QLabel("")
        self.user_lbl.setStyleSheet("""
            background-color: #E5E5EA; 
            color: #1D1D1F; 
            border-radius: 14px; 
            padding: 4px 12px; 
            font-size: 13px; 
            font-weight: 500;
            margin-left: 10px;
        """)
        self.user_lbl.hide() # Hidden by default
        layout.addWidget(self.user_lbl)

    def set_user_id(self, uid):
        if uid:
            self.user_lbl.setText(f"👤 {uid}")
            self.user_lbl.show()
        else:
            self.user_lbl.hide()

class MacPage(QWidget):
    """ Base class for standard pages with margins """
    def __init__(self):
        super().__init__()
        self.navbar = None  # Track navbar
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)
        
        # Content container
        self.content_widget = QWidget()
        self.content_layout = QVBoxLayout(self.content_widget)
        self.content_layout.setContentsMargins(40, 40, 40, 40)
        self.content_layout.setSpacing(20)
        
        self.main_layout.addWidget(self.content_widget)

    def set_navbar(self, navbar):
        if self.navbar:
            self.main_layout.removeWidget(self.navbar)
            self.navbar.deleteLater()
        self.navbar = navbar
        self.main_layout.insertWidget(0, navbar)
        
    def set_user_id(self, uid):
        if self.navbar:
            self.navbar.set_user_id(uid)

# ---------------------------------------------------------
# Page 0: Login
# ---------------------------------------------------------
class PageLogin(MacPage):
    def __init__(self, on_login_success, on_signup):
        super().__init__()
        self.on_login_success = on_login_success
        self.on_signup = on_signup
        
        # Nav
        nav = Navbar("로그인")
        self.set_navbar(nav)
        
        # 1. Login Section
        self.login_card = QFrame()
        self.login_card.setObjectName("Card")
        lc_layout = QVBoxLayout(self.login_card)
        lc_layout.setContentsMargins(30, 30, 30, 30)
        
        title = QLabel("환영합니다")
        title.setObjectName("Title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lc_layout.addWidget(title)
        
        form = QFormLayout()
        form.setSpacing(15)
        self.login_id_edit = QLineEdit()
        self.login_id_edit.setPlaceholderText("user_id")
        self.pw_edit = QLineEdit()
        self.pw_edit.setPlaceholderText("••••••••")
        self.pw_edit.setEchoMode(QLineEdit.EchoMode.Password)
        
        form.addRow("아이디", self.login_id_edit)
        form.addRow("비밀번호", self.pw_edit)
        lc_layout.addLayout(form)
        
        # Buttons Row
        btn_row = QHBoxLayout()
        
        btn_signup = QPushButton("회원가입")
        btn_signup.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_signup.clicked.connect(self.on_signup)
        
        btn_login = QPushButton("로그인 →")
        btn_login.setObjectName("Primary")
        btn_login.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_login.clicked.connect(self._handle_login)
        
        btn_row.addWidget(btn_signup)
        btn_row.addStretch()
        btn_row.addWidget(btn_login)
        
        lc_layout.addLayout(btn_row)
        
        # Layout
        self.content_layout.addStretch()
        self.content_layout.addWidget(self.login_card)
        self.content_layout.addStretch()

    def _handle_login(self):
        login_id = self.login_id_edit.text().strip()
        pw = self.pw_edit.text().strip()
        if not login_id or not pw:
            QMessageBox.warning(self, "입력 부족", "아이디와 비밀번호를 입력해주세요.")
            return
        
        # DB Verification
        conn = get_db_connection()
        cursor = conn.cursor()
        try:
            # 해싱된 비밀번호 비교 (실제로는 salt등 더 안전한 방법 필요)
            hashed_pw = hashlib.sha256(pw.encode()).hexdigest()
            
            cursor.execute("SELECT id, name FROM users WHERE login_id = %s AND password = %s", (login_id, hashed_pw))
            user = cursor.fetchone()
            
            if user:
                # 로그인 성공
                print(f"Login success: {user['name']} ({user['id']})")
                self.on_login_success(login_id, user['id']) # user_id(DB index)도 전달
            else:
                QMessageBox.warning(self, "로그인 실패", "아이디 또는 비밀번호가 올바르지 않습니다.")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"로그인 중 오류가 발생했습니다: {e}")
        finally:
            conn.close()

# ---------------------------------------------------------
# Page 5: Signup
# ---------------------------------------------------------
class PageSignup(MacPage):
    def __init__(self, nav_manager, on_save_user):
        super().__init__()
        self.nav = nav_manager
        self.on_save_user = on_save_user
        
        # Nav
        btn_back = QPushButton("← 로그인")
        btn_back.clicked.connect(lambda: self.nav.go(0))
        self.set_navbar(Navbar("회원가입", [btn_back]))
        
        self.card = QFrame()
        self.card.setObjectName("Card")
        layout = QVBoxLayout(self.card)
        layout.setContentsMargins(30,30,30,30)
        
        title = QLabel("회원가입")
        title.setObjectName("Subtitle")
        layout.addWidget(title)
        
        u_form = QFormLayout()
        u_form.setSpacing(15)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("홍길동")
        
        self.is_id_checked = False  # ID 중복확인 완료 여부
        
        # ID Field with Duplicate Check
        id_layout = QHBoxLayout()
        # id_layout.setContentsMargins(0, 0, 0, 0)
        self.uid_edit = QLineEdit()
        self.uid_edit.setPlaceholderText("user123")
        # ID 변경 시 중복확인 상태 초기화
        self.uid_edit.textChanged.connect(lambda: setattr(self, 'is_id_checked', False))
        
        btn_check = QPushButton("중복확인")
        btn_check.setFixedWidth(100)
        btn_check.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_check.clicked.connect(self.check_duplicate_id)
        
        id_layout.addWidget(self.uid_edit)
        id_layout.addWidget(btn_check)
        
        self.upw_edit = QLineEdit()
        self.upw_edit.setPlaceholderText("비밀번호 (영문/숫자/특수문자)")
        self.upw_edit.setEchoMode(QLineEdit.EchoMode.Password)
        # ASCII Only Validator
        ascii_regex = QRegularExpression(r"[\x21-\x7E]+")
        validator = QRegularExpressionValidator(ascii_regex)
        self.upw_edit.setValidator(validator)
        
        self.upw_confirm_edit = QLineEdit()
        self.upw_confirm_edit.setPlaceholderText("비밀번호 확인")
        self.upw_confirm_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.upw_confirm_edit.setValidator(validator)
        
        self.birth_edit = QLineEdit()
        self.birth_edit.setPlaceholderText("YYYYMMDD")
        
        u_form.addRow("이름", self.name_edit)
        u_form.addRow("아이디", id_layout)
        u_form.addRow("비밀번호", self.upw_edit)
        u_form.addRow("비밀번호 확인", self.upw_confirm_edit)
        u_form.addRow("생년월일", self.birth_edit)
        layout.addLayout(u_form)
        
        btn_save = QPushButton("정보 저장")
        btn_save.setObjectName("Primary")
        btn_save.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_save.clicked.connect(self._handle_save)
        layout.addWidget(btn_save, alignment=Qt.AlignmentFlag.AlignRight)
        
        self.content_layout.addStretch()
        self.content_layout.addWidget(self.card)
        self.content_layout.addStretch()

    def check_duplicate_id(self):
        uid = self.uid_edit.text().strip()
        if not uid:
            QMessageBox.warning(self, "입력 오류", "아이디를 입력해주세요.")
            return

        # Simple Regex Check
        if not re.match(r"^[a-zA-Z0-9]+$", uid):
            QMessageBox.warning(self, "입력 오류", "아이디는 영어(문자/숫자)로 입력해주세요.")
            return

        conn = get_db_connection()
        cursor = conn.cursor()
        try:
            cursor.execute("SELECT id FROM users WHERE login_id = %s", (uid,))
            if cursor.fetchone():
                self.is_id_checked = False
                QMessageBox.warning(self, "불가", "이미 존재하는 아이디입니다.")
            else:
                self.is_id_checked = True
                QMessageBox.information(self, "가능", "사용 가능한 아이디입니다.")
        except Exception as e:
            self.is_id_checked = False
            QMessageBox.critical(self, "오류", f"중복 확인 중 오류: {e}")
        finally:
            conn.close()

    def _handle_save(self):
        name = self.name_edit.text().strip()
        uid = self.uid_edit.text().strip()
        upw = self.upw_edit.text().strip()
        upw_confirm = self.upw_confirm_edit.text().strip()
        birth = self.birth_edit.text().strip()

        # 1. Empty Checks
        if not name:
            QMessageBox.warning(self, "입력 오류", "이름을 입력해주세요.")
            return
        if not uid:
            QMessageBox.warning(self, "입력 오류", "아이디를 입력해주세요.")
            return
        if not upw:
            QMessageBox.warning(self, "입력 오류", "비밀번호를 입력해주세요.")
            return
        if not upw_confirm:
            QMessageBox.warning(self, "입력 오류", "비밀번호 확인을 입력해주세요.")
            return
        if not birth:
            QMessageBox.warning(self, "입력 오류", "생년월일을 입력해주세요.")
            return
            
        # 1.3 ID Check verify
        if not self.is_id_checked:
            QMessageBox.warning(self, "확인 필요", "아이디 중복확인을 먼저 진행해주세요.")
            return
            
        # 1.5 Password Match Check
        if upw != upw_confirm:
            QMessageBox.warning(self, "비밀번호 불일치", "비밀번호가 일치하지 않습니다.\n다시 확인해주세요.")
            return

        # 2. ID Validation (English/Number only)
        if not re.match(r"^[a-zA-Z0-9]+$", uid):
            QMessageBox.warning(self, "입력 오류", "아이디는 영어(문자/숫자)로 입력해주세요.")
            return

        # 3. Password Validation (8+ chars, English + Special)
        # Regex: At least 1 English char ([a-zA-Z]), At least 1 Special char (not alphanumeric/space), Length >= 8
        if len(upw) < 8 or not re.search(r"[a-zA-Z]", upw) or not re.search(r"[^a-zA-Z0-9]", upw):
            QMessageBox.warning(self, "입력 오류", "비밀번호는 영어와 특수문자를 포함하여\n8자리 이상이어야 합니다.")
            return

        # 4. Birthdate Validation
        if not re.match(r"^\d{8}$", birth):
            QMessageBox.warning(self, "입력 오류", "생년월일은 8자리 숫자로 입력해주세요.\n예: 19900101")
            return

        # 5. DB Save
        conn = get_db_connection()
        cursor = conn.cursor()
        try:
            # Check duplicate ID
            cursor.execute("SELECT id FROM users WHERE login_id = %s", (uid,))
            if cursor.fetchone():
                QMessageBox.warning(self, "중복 오류", "이미 존재하는 아이디입니다.")
                return
            
            # Check duplicate Name + Birth
            cursor.execute("SELECT id FROM users WHERE name = %s AND birth_date = %s", (name, birth))
            if cursor.fetchone():
                QMessageBox.warning(self, "가입 불가", "이미 등록된 회원입니다.\n(이름과 생년월일이 동일한 계정이 존재합니다)")
                return

            hashed_pw = hashlib.sha256(upw.encode()).hexdigest()
            
            # Insert into users
            # age, address 등 필수 컬럼이 있다면 기본값 처리 필요
            # 여기서는 age=0, address='Unknown'으로 설정 (스키마에 따라 수정 필요)
            cursor.execute("""
                INSERT INTO users (name, login_id, password, birth_date, age, address, created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s, %s, NOW(), NOW())
            """, (name, uid, hashed_pw, birth, 20, "Unknown")) # age 임시값 20
            
            conn.commit()
            
            # on_save_user 콜백 호출 (필요시)
            data = {
                "name": name,
                "id": uid,
                "birth": birth
            }
            self.on_save_user(data)
            
            QMessageBox.information(self, "저장 완료", "회원가입이 완료되었습니다.\n로그인 페이지로 이동합니다.")
            self.nav.go(0)
            
        except Exception as e:
            if 'conn' in locals():
                conn.rollback()
            QMessageBox.critical(self, "저장 오류", f"회원가입 중 오류가 발생했습니다:\n{e}")
        finally:
            if 'conn' in locals():
                conn.close()

# ---------------------------------------------------------
# Page 1: Products
# ---------------------------------------------------------
class ProductItem(QFrame):
    def __init__(self, product, add_callback):
        super().__init__()
        self.setObjectName("Card")
        self.setFixedSize(220, 320)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Image
        img_lbl = QLabel()
        img_lbl.setPixmap(make_dummy_image(product.name, 190, 140))
        img_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(img_lbl)
        
        # Info
        name = QLabel(product.name)
        name.setStyleSheet("font-weight: 700; font-size: 15px;")
        name.setWordWrap(True)
        layout.addWidget(name)
        
        price = QLabel(f"₩{product.price:,}")
        price.setStyleSheet("color: #1D1D1F; font-size: 14px;")
        layout.addWidget(price)
        
        stock = QLabel(product.stock)
        color = "#34C759" if "In" in product.stock else "#FF9500"
        stock.setStyleSheet(f"color: {color}; font-size: 12px; font-weight: 600;")
        layout.addWidget(stock)
        
        layout.addStretch()
        
        # Controls
        row = QHBoxLayout()
        self.spin = QSpinBox()
        self.spin.setRange(1, 10)
        self.spin.setFixedWidth(50)
        
        btn = QPushButton("+ 담기")
        btn.setObjectName("Primary")
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.setFixedHeight(30)
        btn.clicked.connect(lambda: add_callback(product, self.spin.value()))
        
        row.addWidget(self.spin)
        row.addWidget(btn)
        layout.addLayout(row)

class PageProduct(MacPage):
    def __init__(self, nav_manager, get_cart, update_cart):
        super().__init__()
        self.nav = nav_manager
        self.get_cart = get_cart
        self.update_cart = update_cart
        
        # Top Nav
        btn_home = QPushButton("🏠 홈")
        btn_home.clicked.connect(lambda: self.nav.go(0))
        
        self.btn_next = QPushButton("결제하기 →")
        self.btn_next.clicked.connect(lambda: self.nav.go(2))
        
        navbar = Navbar("상품 보기", [btn_home], [self.btn_next])
        self.set_navbar(navbar)
        
        # Layout: Left (Grid) + Right (Cart)
        h_layout = QHBoxLayout()
        h_layout.setSpacing(20)
        
        # Left: Scroll Area for Grid
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        self.grid_container = QWidget()
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setSpacing(16)
        self.grid_layout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        
        # Load Products
        self.load_products_from_db()
        
        scroll.setWidget(self.grid_container)
        h_layout.addWidget(scroll, 3)
        
        # Right: Cart
        cart_panel = QFrame()
        cart_panel.setObjectName("Card")
        cart_v = QVBoxLayout(cart_panel)
        cart_v.setContentsMargins(20, 20, 20, 20)
        
        cart_title = QLabel("장바구니")
        cart_title.setObjectName("Subtitle")
        cart_v.addWidget(cart_title)
        
        self.cart_list = QListWidget()
        cart_v.addWidget(self.cart_list)
        
        self.total_lbl = QLabel("합계: ₩0")
        self.total_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.total_lbl.setStyleSheet("font-weight: 700; font-size: 16px;")
        cart_v.addWidget(self.total_lbl)
        
        del_btn = QPushButton("선택 삭제")
        del_btn.clicked.connect(self.remove_selected)
        cart_v.addWidget(del_btn)
        
        h_layout.addWidget(cart_panel, 1)
        self.content_layout.addLayout(h_layout)

    def load_products_from_db(self):
        conn = get_db_connection()
        cursor = conn.cursor()
        try:
            # 1. 쿼리에 marker_id 추가
            cursor.execute("SELECT id, name, price, stock, category, marker_id FROM products")
            rows = cursor.fetchall()
            
            products = []
            for row in rows:
                stock_status = "In Stock"
                if row['stock'] == 0:
                    stock_status = "Out of Stock"
                elif row['stock'] < 5:
                    stock_status = "Low Stock"
                    
                p = Product(row['name'], row['price'], stock_status)
                # (Product객체, id, marker_id) 튜플로 저장
                products.append((p, row['id'], row.get('marker_id')))
            
            # Grid 배치
            # 기존 아이템 제거 (재로딩 시)
            for i in reversed(range(self.grid_layout.count())): 
                self.grid_layout.itemAt(i).widget().setParent(None)

            r, c = 0, 0
            r, c = 0, 0
            # 수정된 튜플 언패킹 (p, pid, mid)
            for p, pid, mid in products:
                # lambda에 mid(marker_id) 추가
                item = ProductItem(p, lambda prod, q, pid=pid, mid=mid: self.add_to_cart(prod, q, pid, mid))
                self.grid_layout.addWidget(item, r, c)
                c += 1
                if c > 2:
                    c = 0
                    r += 1
                    
        except Exception as e:
            print(f"Failed to load products: {e}")
            lbl = QLabel(f"제품 로드 실패: {e}")
            self.grid_layout.addWidget(lbl, 0, 0)
        finally:
            conn.close()

    def add_to_cart(self, product, qty, pid, marker_id):
        cart = self.get_cart() # Assuming get_cart() returns the actual cart list
        # Check existing
        found = False
        for item in cart:
            if item.get('id') == pid: # Use pid for checking existing items
                item['qty'] += qty
                found = True
                break
        
        if not found:
            cart.append({
                "id": pid,
                "name": product.name, 
                "price": product.price, 
                "qty": qty,
                "marker_id": marker_id # Add marker_id to the cart item
            })
        self.update_cart(cart) # Assuming update_cart takes the modified cart
        self.refresh_cart_ui()
        QMessageBox.information(self, "장바구니", f"{product.name}이(가) 장바구니에 담겼습니다.")
        
    def remove_selected(self):
        row = self.cart_list.currentRow()
        if row >= 0:
            cart = self.get_cart()
            del cart[row]
            self.update_cart(cart)
            self.refresh_cart_ui()
            
    def refresh_cart_ui(self):
        self.cart_list.clear()
        cart = self.get_cart()
        total = 0
        for item in cart:
            sub = item['price'] * item['qty']
            total += sub
            self.cart_list.addItem(f"{item['name']} x{item['qty']} (₩{sub:,})")
        self.total_lbl.setText(f"합계: ₩{total:,}")

# ---------------------------------------------------------
# Page 2: Placeholder
# ---------------------------------------------------------


# ---------------------------------------------------------
# Page 2: Placeholder
# ---------------------------------------------------------
class PagePlaceholder(MacPage):
    def __init__(self, nav_manager):
        super().__init__()
        self.nav = nav_manager
        
        # Nav
        btn_home = QPushButton("🏠 홈")
        btn_home.clicked.connect(lambda: self.nav.go(0))
        btn_prev = QPushButton("← 상품")
        btn_prev.clicked.connect(lambda: self.nav.go(1))
        
        btn_next = QPushButton("결제로 →")
        # btn_next.setObjectName("Primary") <-- Removed to match Home button style
        btn_next.clicked.connect(lambda: self.nav.go(3))
        
        self.set_navbar(Navbar("3페이지 (미정)", [btn_home, btn_prev], [btn_next]))
        
        # Content
        box = QFrame()
        box.setObjectName("Card")
        box.setFixedHeight(300)
        
        bl = QVBoxLayout(box)
        bl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        lbl = QLabel("Placeholder Page")
        lbl.setStyleSheet("font-size: 32px; color: #D2D2D7; font-weight: 700;")
        bl.addWidget(lbl)
        
        self.content_layout.addWidget(box)
        self.content_layout.addStretch()

# ---------------------------------------------------------
# Page 3: Checkout
# ---------------------------------------------------------
class PageCheckout(MacPage):
    def __init__(self, nav_manager, get_mem, get_cart, set_receipt):
        super().__init__()
        self.nav = nav_manager
        self.get_mem = get_mem
        self.get_cart = get_cart
        self.set_receipt = set_receipt
        
        # Nav
        btn_home = QPushButton("🏠 홈")
        btn_home.clicked.connect(lambda: self.nav.go(0))
        btn_prev = QPushButton("← 이전")
        btn_prev.clicked.connect(lambda: self.nav.go(1))
        
        self.set_navbar(Navbar("결제", [btn_home, btn_prev]))
        
        # Two columns
        row = QHBoxLayout()
        row.setSpacing(24)
        
        # Left: Info
        left_col = QVBoxLayout()
        
        # User Info
        self.grp_user = QGroupBox("배송 정보 (자동 입력)")
        self.grp_user.setFixedWidth(400)
        form = QFormLayout()
        self.u_name = QLineEdit()
        self.u_id = QLineEdit()
        self.u_birth = QLineEdit()
        form.addRow("이름", self.u_name)
        form.addRow("아이디", self.u_id)
        form.addRow("생년월일", self.u_birth)
        self.grp_user.setLayout(form)
        left_col.addWidget(self.grp_user)
        
        left_col.addStretch()
        row.addLayout(left_col)
        
        # Right: Order Summary
        right_col = QVBoxLayout()
        self.summary_card = QFrame()
        self.summary_card.setObjectName("Card")
        # self.summary_card.setFixedWidth(350)
        sl = QVBoxLayout(self.summary_card)
        sl.setContentsMargins(24, 24, 24, 24)
        
        sl.addWidget(QLabel("주문 내역", objectName="Subtitle"))
        self.order_list = QListWidget()
        self.order_list.setStyleSheet("border: none; background: transparent;")
        sl.addWidget(self.order_list)
        
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #E5E5EA;")
        sl.addWidget(line)
        
        self.total_lbl = QLabel("총 결제금액: ₩0")
        self.total_lbl.setStyleSheet("font-size: 18px; font-weight: 800; color: #007AFF;")
        self.total_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        sl.addWidget(self.total_lbl)
        
        btn_pay = QPushButton("결제하기")
        btn_pay.setObjectName("Primary")
        btn_pay.setFixedHeight(44)
        btn_pay.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_pay.clicked.connect(self.process_payment)
        sl.addWidget(btn_pay)
        
        right_col.addWidget(self.summary_card)
        right_col.addStretch()
        row.addLayout(right_col)
        
        self.content_layout.addLayout(row)

    def ensure_data(self):
        # Load user
        mem = self.get_mem()
        self.u_name.setText(mem.get('name', ''))
        self.u_id.setText(mem.get('id', ''))
        self.u_birth.setText(mem.get('birth', ''))
        
        # Load cart
        self.order_list.clear()
        cart = self.get_cart()
        total = 0
        for item in cart:
            sub = item['price'] * item['qty']
            total += sub
            self.order_list.addItem(f"{item['name']} x {item['qty']}  = ₩{sub:,}")
        self.total_lbl.setText(f"총 결제금액: ₩{total:,}")

    def process_payment(self):
        # Default values since UI is removed
        method = "카드결제"
        note = ""

        # Save to DB
        mem = self.get_mem() # user info
        cart = self.get_cart()
        
        if not cart:
            QMessageBox.warning(self, "장바구니", "장바구니가 비어있습니다.")
            return

        conn = get_db_connection()
        cursor = conn.cursor()
        
        try:
            # 1. login_id로 user_id 조회 (mem에 id가 user_id가 아닌 login_id일 수 있음)
            # handle_login에서 user_id(int)를 넘겨주도록 수정했다면 mem['user_db_id'] 등에 있을 것
            # 혹은 여기서 login_id로 다시 조회
            login_id = mem.get('id')
            cursor.execute("SELECT id FROM users WHERE login_id = %s", (login_id,))
            user_row = cursor.fetchone()
            if not user_row:
                 QMessageBox.warning(self, "오류", "사용자 정보를 찾을 수 없습니다.")
                 return
            user_db_id = user_row['id']
            
            total_price = sum(item['price'] * item['qty'] for item in cart)
            
            # 2. Insert Order
            cursor.execute("""
                INSERT INTO orders (user_id, total_price, status, created_at)
                VALUES (%s, %s, %s, NOW())
            """, (user_db_id, total_price, "PENDING"))
            order_id = cursor.lastrowid
            
            # 3. Insert Order Items & Update Stock
            for item in cart:
                pid = item.get('id')
                # pid가 없으면 name으로 조회해야 함 (기존 더미 호환)
                if not pid:
                     cursor.execute("SELECT id FROM products WHERE name = %s", (item['name'],))
                     p_row = cursor.fetchone()
                     if p_row:
                         pid = p_row['id']
                
                if pid:
                    cursor.execute("""
                        INSERT INTO order_items (order_id, product_id, quantity, price_at_order)
                        VALUES (%s, %s, %s, %s)
                    """, (order_id, pid, item['qty'], item['price']))
                    
                    # Update Stock
                    cursor.execute("""
                        UPDATE products SET stock = stock - %s WHERE id = %s
                    """, (item['qty'], pid))
            
            conn.commit()
            print(f"Order saved: ID {order_id}")
            
            import json # Ensure json is imported for debug logging
            # --- Robot Arm Trigger Start (ROS 2 & SSH Fallback) ---
            marker_ids = []
            for item in cart:
                mid = item.get('marker_id')
                if mid is not None:
                    marker_ids.append(mid)
            
            print(f"DEBUG: Cart items: {json.dumps(cart, ensure_ascii=False)}")
            print(f"DEBUG: Extracted marker_ids: {marker_ids}")

            # Trigger Pinky (Autonomous Mobile Robot) - Trigger ALWAYS on payment
            # BLOCKING CALL to ensure Pinky arrives before arms move
            print("INFO: Triggering Pinky Mission 123 (Arrival) and WAITING...")
            try:
                # Use subprocess.run to wait for completion
                log_file = open("/home/name/server/pinky_trigger.log", "w")
                res = subprocess.run(["/home/name/server/trigger_pinky.sh", "123"], stdout=log_file, stderr=subprocess.STDOUT)
                if res.returncode == 0:
                     print("✅ Pinky Arrived!")
                else:
                     print("⚠️ Pinky trigger returned non-zero exit code")
            except Exception as e:
                print(f"❌ Failed to trigger Pinky: {e}")

            if marker_ids:
                # 로봇팔별 할당된 마커 ID 정의
                arm17_ids = [6, 37]
                arm18_ids = [7]
                
                arm17_selected = [mid for mid in marker_ids if mid in arm17_ids]
                arm18_selected = [mid for mid in marker_ids if mid in arm18_ids]

                print(f"DEBUG: Arm 17 targets: {arm17_selected}")
                print(f"DEBUG: Arm 18 targets: {arm18_selected}")

                # Arm 17 Trigger (ROS 2)
                if arm17_selected:
                    if ROS2_AVAILABLE and ros2_arm17:
                        print(f"[ROS 2] Triggering Arm 17 via Domain 7: {arm17_selected}")
                        ros2_arm17.publish_pick(arm17_selected)
                    else:
                        print("[SSH] Falling back to Arm 17 SSH trigger (ROS 2 Unavailable)")
                        # Fallback logic if needed, but we prefer ROS 2 now
                        arg = "both" if set(arm17_selected) == {6, 37} else str(arm17_selected[0])
                        subprocess.run(["/home/name/server/trigger_arm17.sh", arg], timeout=10)

                # Arm 18 Trigger (ROS 2)
                if arm18_selected:
                    if ROS2_AVAILABLE and ros2_arm18:
                        print(f"[ROS 2] Triggering Arm 18 via Domain 19: {arm18_selected}")
                        ros2_arm18.publish_pick(arm18_selected)
                    else:
                        print("[SSH] Falling back to Arm 18 SSH trigger")
                         # Legacy fallback
                        subprocess.run(["/home/name/server/trigger_arm18.sh", "7"], timeout=10)
            # --- Robot Arm Trigger End ---
            # --- Robot Arm Trigger End ---
            
            # Create Receipt Data
            receipt = {
                "order_id": order_id,
                "user": {
                    "name": self.u_name.text(),
                    "id": self.u_id.text(),
                    "birth": self.u_birth.text(),
                },
                "cart": cart, 
                "payment": {
                    "method": method,
                    "note": note
                }
            }
            self.set_receipt(receipt)
            # 장바구니 비우기
            self.get_cart().clear()
            self.nav.go(5) # Go to Payment Success Page
            
        except Exception as e:
            conn.rollback()
            QMessageBox.critical(self, "주문 오류", f"주문 처리 중 오류가 발생했습니다: {e}")
        finally:
            conn.close()

# ---------------------------------------------------------
# Page 6: Payment Success (Interstitial)
# ---------------------------------------------------------
class PagePaymentSuccess(MacPage):
    def __init__(self, nav_manager):
        super().__init__()
        self.nav = nav_manager
        
        # We don't use standard Navbar here for cleaner look, or maybe just simple one
        self.set_navbar(Navbar("결제 완료"))
        
        self.timer = QTimer(self)
        self.timer.setInterval(10000) # 10 seconds
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._auto_go_home)

        # Content
        col = QVBoxLayout()
        col.setSpacing(30)
        col.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        icon = QLabel("🎉")
        icon.setStyleSheet("font-size: 64px;")
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        col.addWidget(icon)
        
        msg = QLabel("결제완료")
        msg.setStyleSheet("font-size: 32px; font-weight: 800; color: #1D1D1F;")
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        col.addWidget(msg)
        
        sub = QLabel("잠시 후 홈으로 자동 이동합니다...")
        sub.setStyleSheet("font-size: 14px; color: #86868B;")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        col.addWidget(sub)
        
        # Buttons
        btn_cnt = QWidget()
        btn_row = QHBoxLayout(btn_cnt)
        btn_row.setSpacing(20)
        
        b_home = QPushButton("홈")
        b_home.setFixedSize(120, 44)
        b_home.setCursor(Qt.CursorShape.PointingHandCursor)
        b_home.clicked.connect(lambda: self.nav.go(0))
        
        b_receipt = QPushButton("영수증")
        b_receipt.setObjectName("Primary")
        b_receipt.setFixedSize(120, 44)
        b_receipt.setCursor(Qt.CursorShape.PointingHandCursor)
        b_receipt.clicked.connect(lambda: self.nav.go(4))
        
        btn_row.addWidget(b_home)
        btn_row.addWidget(b_receipt)
        col.addWidget(btn_cnt)
        
        self.content_layout.addStretch()
        self.content_layout.addLayout(col)
        self.content_layout.addStretch()

    def showEvent(self, event):
        super().showEvent(event)
        self.timer.start()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.timer.stop()
        
    def _auto_go_home(self):
        # If still visible, go home
        if self.isVisible():
            self.nav.go(0)

# ---------------------------------------------------------
# Page 4: Receipt
# ---------------------------------------------------------
class PageReceipt(MacPage):
    def __init__(self, nav_manager, get_receipt_data):
        super().__init__()
        self.nav = nav_manager
        self.get_data = get_receipt_data
        
        # Nav
        btn_home = QPushButton("🏠 홈")
        btn_home.clicked.connect(lambda: self.nav.go(0))
        # btn_prev = QPushButton("← 결제로") <-- Removed
        # btn_prev.clicked.connect(lambda: self.nav.go(3))
        
        self.set_navbar(Navbar("영수증", [btn_home]))
        
        # Content centered
        center_box = QFrame()
        center_box.setObjectName("Card")
        center_box.setFixedWidth(500)
        
        self.cl = QVBoxLayout(center_box)
        self.cl.setContentsMargins(40, 40, 40, 40)
        self.cl.setSpacing(10)
        
        # Icon
        icon = QLabel("✅")
        icon.setStyleSheet("font-size: 48px;")
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cl.addWidget(icon)
        
        title = QLabel("결제가 완료되었습니다!")
        title.setObjectName("Subtitle")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cl.addWidget(title)
        
        self.info_lbl = QLabel()
        self.info_lbl.setWordWrap(True)
        self.cl.addWidget(self.info_lbl)
        
        self.cl.addStretch()
        
        wrapper = QHBoxLayout()
        wrapper.addStretch()
        wrapper.addWidget(center_box)
        wrapper.addStretch()
        
        self.content_layout.addLayout(wrapper)
        self.content_layout.addStretch()

    def render(self):
        data = self.get_data()
        if not data:
            self.info_lbl.setText("데이터가 없습니다.")
            return
            
        u = data.get('user', {})
        p = data.get('payment', {})
        cart = data.get('cart', [])
        
        total = sum(i['price'] * i['qty'] for i in cart)
        
        txt = f"""
        <hr>
        <b>구매자:</b> {u.get('name')} <br>
        <b>아이디:</b> {u.get('id')} <br>
        <b>생년월일:</b> {u.get('birth')} <br>
        <hr>
        <b>결제수단:</b> {p.get('method')} <br>
        <b>요청사항:</b> {p.get('note')} <br>
        <hr>
        <b>구매품목:</b> {len(cart)}종류 <br>
        <b>총 결제금액:</b> <span style='font-size:16px; color:#007AFF'>₩{total:,}</span>
        """
        self.info_lbl.setText(txt)

# ---------------------------------------------------------
# Page 6: Admin (LLM Control)
# ---------------------------------------------------------
class PageAdmin(MacPage):
    def __init__(self, nav_manager):
        super().__init__()
        self.nav = nav_manager
        
        btn_home = QPushButton("🏠 홈")
        btn_home.clicked.connect(lambda: self.nav.go(0))
        self.set_navbar(Navbar("시스템 관리자 (LLM)", [btn_home]))
        
        # 1. Input Section
        self.card = QFrame()
        self.card.setObjectName("Card")
        layout = QVBoxLayout(self.card)
        layout.setContentsMargins(30, 30, 30, 30)
        
        lbl = QLabel("로봇에게 자연어 명령을 내리세요:")
        lbl.setObjectName("Subtitle")
        layout.addWidget(lbl)
        
        self.cmd_edit = QLineEdit()
        self.cmd_edit.setPlaceholderText("예: LED를 파란색으로 켜고 앞으로 50cm 가줘")
        self.cmd_edit.returnPressed.connect(self.send_llm_command)
        layout.addWidget(self.cmd_edit)
        
        self.btn_send = QPushButton("명령 전송")
        self.btn_send.setObjectName("Primary")
        self.btn_send.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_send.clicked.connect(self.send_llm_command)
        layout.addWidget(self.btn_send, alignment=Qt.AlignmentFlag.AlignRight)
        
        # 2. Server/Robot Status Section
        status_box = QGroupBox("시스템 상태")
        status_layout = QHBoxLayout(status_box)
        
        self.status_labels = {}
        for name in ["Arm-17", "Arm-18", "Pinky"]:
            lbl = QLabel(f"🔴 {name}")
            lbl.setStyleSheet("color: #FF3B30; font-weight: bold; border: 1px solid #333; padding: 5px; border-radius: 5px;")
            status_layout.addWidget(lbl)
            self.status_labels[name] = lbl
            
        # Refresh Button
        btn_refresh = QPushButton("🔄")
        btn_refresh.setFixedSize(30, 30)
        btn_refresh.clicked.connect(self.check_status)
        status_layout.addWidget(btn_refresh)
        
        layout.addWidget(status_box)
        
        # Timer for auto-refresh status
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(5000)
        self.status_timer.timeout.connect(self.check_status)
        self.status_timer.start()

        # 3. Output Section
        log_lbl = QLabel("실행 로그:")
        log_lbl.setStyleSheet("font-weight: 600; margin-top: 10px;")
        layout.addWidget(log_lbl)
        
        from PyQt6.QtWidgets import QTextEdit
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("background-color: #1E1E1E; color: #D4D4D4; font-family: 'Courier New'; font-size: 13px;")
        layout.addWidget(self.log_view)
        
        self.content_layout.addWidget(self.card)

    def log(self, text, color="#D4D4D4"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"<span style='color:#86868B;'>[{timestamp}]</span> <span style='color:{color};'>{text}</span>")

    def send_llm_command(self):
        text = self.cmd_edit.text().strip()
        if not text: return
        
        self.cmd_edit.clear()
        self.log(f"User Query: {text}", "#007AFF")
        self.btn_send.setEnabled(False)
        
        # API Call (FastAPI)
        import json
        import urllib.request
        import ssl
        
        try:
            url = f"{API_SERVER_URL}/api/llm/command"
            self.log(f"Calling API: {url}", "#86868B")
            
            data = json.dumps({"text": text}).encode('utf-8')
            req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
            
            # Use a non-verifying SSL context to avoid potential certificate issues
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            
            with urllib.request.urlopen(req, data=data, context=ctx, timeout=15) as response:
                self.log("Response received from server", "#86868B")
                res_data = json.loads(response.read().decode('utf-8'))
                
                model = res_data.get("model_used", "unknown")
                action = res_data.get("action_json", {})
                classification = res_data.get("classification", "")
                
                self.log(f"Model: {model} | Class: {classification}", "#34C759")
                self.log(f"Plan: {json.dumps(action, indent=2, ensure_ascii=False)}")
                
                # Trigger Robot
                import subprocess
                self.log("Triggering robot execution...", "#86868B")
                trigger_res = subprocess.run(
                    ["/home/name/server/trigger_action.sh", json.dumps(action)],
                    capture_output=True, text=True, timeout=30
                )
                
                if trigger_res.returncode == 0:
                    self.log("✅ Robot execution triggered", "#34C759")
                    if trigger_res.stdout:
                        self.log(f"Robot Info: {trigger_res.stdout.strip()}", "#86868B")
                    if trigger_res.stderr:
                        # Some logs might go to stderr in ROS 2
                        self.log(f"Robot Debug: {trigger_res.stderr.strip()}", "#86868B")
                else:
                    self.log(f"❌ Execution failed: {trigger_res.stderr}", "#FF3B30")
                    
        except Exception as e:
            self.log(f"❌ Error: {e}", "#FF3B30")
        finally:
            self.btn_send.setEnabled(True)

            self.btn_send.setEnabled(True)

    def showEvent(self, event):
        super().showEvent(event)
        self.check_status()

    def check_status(self):
        # A simple check to see if robots are reachable (Ping or ROS)
        # For this demo, we'll check Ping for Arm 17/Pinky and ROS 2 for Arm 18 if possible
        
        # Helper to set label
        def set_status(name, is_ok):
            lbl = self.status_labels.get(name)
            if lbl:
                if is_ok:
                    lbl.setText(f"🟢 {name}")
                    lbl.setStyleSheet("color: #34C759; font-weight: bold; border: 1px solid #333; padding: 5px; border-radius: 5px;")
                else:
                    lbl.setText(f"🔴 {name}")
                    lbl.setStyleSheet("color: #FF3B30; font-weight: bold; border: 1px solid #333; padding: 5px; border-radius: 5px;")

        import subprocess
        
        def ping(host):
            try:
                subprocess.check_call(["ping", "-c", "1", "-W", "1", host], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except:
                return False

        # Check in a separate thread to not freeze UI
        def _check():
            arm17_ok = ping(os.environ.get("ARM17_IP", "192.168.0.xxx"))
            arm18_ok = ping(os.environ.get("ARM18_IP", "192.168.0.xxx"))
            pinky_ok = ping(os.environ.get("PINKY_IP", "192.168.0.xxx"))
            
            # Update UI via signal (or simple QTimer.singleShot for thread safety in simple apps)
            # using QTimer.singleShot to update UI from main thread
            QTimer.singleShot(0, lambda: set_status("Arm-17", arm17_ok))
            QTimer.singleShot(0, lambda: set_status("Arm-18", arm18_ok))
            QTimer.singleShot(0, lambda: set_status("Pinky", pinky_ok))

        threading.Thread(target=_check, daemon=True).start()
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mac-ish Shop Redesign")
        self.resize(1100, 750)
        
        # Data
        self.user_memory = {}
        self.cart_memory = []
        self.receipt_data = {}
        self.current_user_id = ""  # Logged in ID
        
        # Stack
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        
        # Pages
        self.page0 = PageLogin(
            on_login_success=self.handle_login,
            on_signup=lambda: self.go(4)
        )
        self.page1 = PageProduct(
            nav_manager=self,
            get_cart=lambda: self.cart_memory,
            update_cart=self.update_cart
        )
        # Page2 (Former 3): Checkout
        self.page2 = PageCheckout(
            nav_manager=self,
            get_mem=lambda: self.user_memory,
            get_cart=lambda: self.cart_memory,
            set_receipt=self.set_receipt
        )
        # Page3 (Former 4): Receipt
        self.page3 = PageReceipt(
            nav_manager=self,
            get_receipt_data=lambda: self.receipt_data
        )
        # Page4 (Former 5): Signup
        self.page4 = PageSignup(
            nav_manager=self,
            on_save_user=self.save_user_info
        )
        # Page5 (Former 6): Payment Success
        self.page5 = PagePaymentSuccess(nav_manager=self)
        
        # Page6: Admin
        self.page6 = PageAdmin(nav_manager=self)
        
        self.stack.addWidget(self.page0) # 0
        self.stack.addWidget(self.page1) # 1
        self.stack.addWidget(self.page2) # 2
        self.stack.addWidget(self.page3) # 3
        self.stack.addWidget(self.page4) # 4
        self.stack.addWidget(self.page5) # 5
        self.stack.addWidget(self.page6) # 6
        
        self.go(0)

    def handle_login(self, uid, db_id=None):
        self.current_user_id = uid
        # Admin check: if login_id starts with 'ros2' or matches exactly
        if uid.lower() == 'ros2':
            # Admin Logic: Redirect to PageAdmin (6) directly
            # Re,ove Product Info -> Just go to Admin Page
            self.go(6)
            return
        else:
             # Reset Navbar for normal users
            btn_home = QPushButton("🏠 홈")
            btn_home.clicked.connect(lambda: self.go(0))
            btn_next = QPushButton("결제하기 →")
            btn_next.clicked.connect(lambda: self.go(2))
            new_nav = Navbar("상품 보기", [btn_home], [btn_next])
            self.page1.set_navbar(new_nav)
            # Normal users go to Product Page
            self.go(1)

        # DB ID 저장 (필요시)
        if db_id:
             self.user_memory['user_db_id'] = db_id
        # 기본 정보 로드 (이름 등)
        self.user_memory['id'] = uid
        
        # Load extra user info from DB if needed
        conn = get_db_connection()
        try:
             cursor = conn.cursor()
             # login_id로 조회하도록 변경 (uid가 login_id임)
             cursor.execute("SELECT name, birth_date FROM users WHERE login_id = %s", (uid,))
             row = cursor.fetchone()
             if row:
                 self.user_memory['name'] = row['name']
                 self.user_memory['birth'] = row['birth_date']
        except Exception:
            pass
        finally:
            conn.close()
            
            self.user_memory['id'] = uid
        
        # Load extra user info from DB if needed
        conn = get_db_connection()
        try:
             cursor = conn.cursor()
             # login_id로 조회하도록 변경 (uid가 login_id임)
             cursor.execute("SELECT name, birth_date FROM users WHERE login_id = %s", (uid,))
             row = cursor.fetchone()
             if row:
                 self.user_memory['name'] = row['name']
                 self.user_memory['birth'] = row['birth_date']
        except Exception:
            pass
        finally:
            conn.close()

    def go(self, idx):
        if idx == 2:
            self.page2.ensure_data()
        if idx == 3:
            self.page3.render()
        
        self.stack.setCurrentIndex(idx)
        
        # Update User ID in Navbar
        # Don't show on Login (0) or Signup (4)
        page = self.stack.currentWidget()
        if isinstance(page, MacPage):
            if idx in [0, 4]:
                page.set_user_id("")
            else:
                page.set_user_id(self.current_user_id)

    def save_user_info(self, data):
        self.user_memory = data
    
    def update_cart(self, new_cart):
        self.cart_memory = new_cart
        
    def set_receipt(self, data):
        self.receipt_data = data

def main():
    # 1. DB Init & Migrate
    init_db_and_migrate()
    
    app = QApplication(sys.argv)
    # Apply Global Stylesheet
    app.setStyleSheet(STYLESHEET)
    
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
