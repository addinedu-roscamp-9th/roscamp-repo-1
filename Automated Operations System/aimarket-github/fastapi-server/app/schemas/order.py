from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from app.models.order import OrderStatus


class OrderItemCreate(BaseModel):
    product_id: int
    quantity: int = Field(ge=1, le=2, description="Quantity must be between 1 and 2")


class OrderItemOut(BaseModel):
    id: int
    product_id: int
    quantity: int
    price_at_order: int

    class Config:
        from_attributes = True


class OrderCreate(BaseModel):
    user_id: int
    items: list[OrderItemCreate] = Field(min_length=1, description="At least one item required")

    @field_validator("items")
    @classmethod
    def validate_items(cls, items: list[OrderItemCreate]) -> list[OrderItemCreate]:
        if not items:
            raise ValueError("Order must contain at least one item")
        return items


class OrderOut(BaseModel):
    id: int
    user_id: int
    total_price: int
    status: OrderStatus
    created_at: datetime
    items: list[OrderItemOut]

    class Config:
        from_attributes = True


class OrderStatusUpdate(BaseModel):
    status: OrderStatus
