from sqlalchemy import Integer, String, Text, Enum as SQLEnum
from sqlalchemy.orm import Mapped, mapped_column
import enum
from app.db.base import Base


class ProductCategory(str, enum.Enum):
    # 식료품
    VEGETABLE = "vegetable"  # 채소
    FISH = "fish"  # 생선
    FRUIT = "fruit"  # 과일
    MEAT = "meat"  # 고기
    # 의류
    HAT = "hat"  # 모자
    PANTS = "pants"  # 하의
    TOP = "top"  # 상의
    SHOES = "shoes"  # 신발


class Product(Base):
    __tablename__ = "products"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    name: Mapped[str] = mapped_column(String(100), nullable=False)
    category: Mapped[ProductCategory] = mapped_column(SQLEnum(ProductCategory), nullable=False)
    price: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    stock: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    description: Mapped[str | None] = mapped_column(Text, nullable=True)
    marker_id: Mapped[int | None] = mapped_column(Integer, nullable=True, unique=True, index=True)
