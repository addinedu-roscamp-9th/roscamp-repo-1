from pydantic import BaseModel, Field
from app.models.product import ProductCategory


class ProductBase(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    category: ProductCategory
    price: int = Field(ge=0, description="price in KRW")
    stock: int = Field(ge=0, description="stock quantity")
    description: str | None = Field(default=None, max_length=500)
    marker_id: int | None = Field(default=None, description="ArUco marker ID for product tracking")


class ProductCreate(ProductBase):
    pass


class ProductUpdate(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=100)
    category: ProductCategory | None = None
    price: int | None = Field(default=None, ge=0, description="price in KRW")
    stock: int | None = Field(default=None, ge=0, description="stock quantity")
    description: str | None = Field(default=None, max_length=500)
    marker_id: int | None = Field(default=None, description="ArUco marker ID for product tracking")


class ProductOut(ProductBase):
    id: int

    class Config:
        from_attributes = True
