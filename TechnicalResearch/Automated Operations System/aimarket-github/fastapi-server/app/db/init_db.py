"""
Database initialization script
Creates tables and populates with sample data
"""
from sqlalchemy.orm import Session
from app.db.session import engine
from app.db.base import Base

# Import all models explicitly to ensure they are registered
from app.models.user import User
from app.models.product import Product, ProductCategory
from app.models.order import Order, OrderItem
from app.models.delivery import Delivery
from app.models.notification import Notification
from app.models.log import Log


def init_db() -> None:
    """Initialize database with tables and sample data"""
    # Create all tables
    Base.metadata.create_all(bind=engine)
    print("✓ Database tables created")
    
    # Create a session
    from app.db.session import SessionLocal
    db = SessionLocal()
    
    try:
        # Check if products already exist
        existing_products = db.query(Product).first()
        if existing_products:
            print("✓ Sample data already exists, skipping initialization")
            return
        
        # Create sample products
        sample_products = [
            # 식료품 - 채소
            Product(name="당근", category=ProductCategory.VEGETABLE, price=3000, stock=10, description="신선한 당근"),
            Product(name="양파", category=ProductCategory.VEGETABLE, price=2500, stock=15, description="국산 양파"),
            
            # 식료품 - 생선
            Product(name="고등어", category=ProductCategory.FISH, price=8000, stock=8, description="신선한 고등어"),
            Product(name="연어", category=ProductCategory.FISH, price=15000, stock=5, description="노르웨이 연어"),
            
            # 식료품 - 과일
            Product(name="사과", category=ProductCategory.FRUIT, price=5000, stock=20, description="국산 사과"),
            Product(name="바나나", category=ProductCategory.FRUIT, price=4000, stock=25, description="필리핀 바나나"),
            
            # 식료품 - 고기
            Product(name="소고기", category=ProductCategory.MEAT, price=25000, stock=6, description="한우 등심"),
            Product(name="닭고기", category=ProductCategory.MEAT, price=8000, stock=12, description="국내산 닭고기"),
            
            # 의류 - 모자
            Product(name="야구모자", category=ProductCategory.HAT, price=15000, stock=10, description="심플한 야구모자"),
            Product(name="비니", category=ProductCategory.HAT, price=12000, stock=8, description="겨울용 비니"),
            
            # 의류 - 하의
            Product(name="청바지", category=ProductCategory.PANTS, price=45000, stock=15, description="스트레치 청바지"),
            Product(name="슬랙스", category=ProductCategory.PANTS, price=35000, stock=10, description="정장 슬랙스"),
            
            # 의류 - 상의
            Product(name="티셔츠", category=ProductCategory.TOP, price=20000, stock=20, description="면 티셔츠"),
            Product(name="셔츠", category=ProductCategory.TOP, price=30000, stock=12, description="정장 셔츠"),
            
            # 의류 - 신발
            Product(name="운동화", category=ProductCategory.SHOES, price=80000, stock=8, description="러닝화"),
            Product(name="구두", category=ProductCategory.SHOES, price=90000, stock=6, description="정장 구두"),
        ]
        
        db.bulk_save_objects(sample_products)
        db.commit()
        print(f"✓ Created {len(sample_products)} sample products")
        
        # Create sample users
        sample_users = [
            User(name="김철수", age=30, address="서울시 강남구 테헤란로 123"),
            User(name="이영희", age=25, address="서울시 송파구 올림픽로 456"),
            User(name="박민수", age=35, address="서울시 마포구 월드컵로 789"),
        ]
        
        db.bulk_save_objects(sample_users)
        db.commit()
        print(f"✓ Created {len(sample_users)} sample users")
        
        print("✓ Database initialized successfully!")
        
    except Exception as e:
        print(f"✗ Error initializing database: {e}")
        db.rollback()
        raise
    finally:
        db.close()


if __name__ == "__main__":
    init_db()
