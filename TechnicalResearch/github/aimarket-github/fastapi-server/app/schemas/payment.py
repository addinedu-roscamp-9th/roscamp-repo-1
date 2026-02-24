from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional
from app.models.payment import PaymentMethod, PaymentStatus


# Payment Schemas
class PaymentBase(BaseModel):
    order_id: int
    amount: int = Field(..., gt=0, description="결제 금액 (양수)")
    method: PaymentMethod


class PaymentCreate(PaymentBase):
    pass


class PaymentUpdate(BaseModel):
    status: Optional[PaymentStatus] = None
    transaction_id: Optional[str] = None
    pg_response: Optional[dict] = None


class PaymentResponse(PaymentBase):
    id: int
    status: PaymentStatus
    transaction_id: Optional[str] = None
    pg_response: Optional[dict] = None
    created_at: datetime
    completed_at: Optional[datetime] = None

    class Config:
        from_attributes = True
