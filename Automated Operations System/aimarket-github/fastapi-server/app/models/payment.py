from datetime import datetime
from sqlalchemy import Integer, String, Float, DateTime, ForeignKey, Enum as SQLEnum, Text, JSON
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy.sql import func
import enum
from app.db.base import Base


class PaymentMethod(str, enum.Enum):
    CARD = "card"            # 카드 결제
    CASH = "cash"            # 현금 결제
    POINT = "point"          # 포인트 결제
    TRANSFER = "transfer"    # 계좌이체


class PaymentStatus(str, enum.Enum):
    PENDING = "pending"          # 결제 대기
    COMPLETED = "completed"      # 결제 완료
    FAILED = "failed"            # 결제 실패
    REFUNDED = "refunded"        # 환불 완료
    CANCELLED = "cancelled"      # 결제 취소


class Payment(Base):
    __tablename__ = "payments"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    order_id: Mapped[int] = mapped_column(Integer, ForeignKey("orders.id"), nullable=False)
    amount: Mapped[int] = mapped_column(Integer, nullable=False)
    method: Mapped[PaymentMethod] = mapped_column(SQLEnum(PaymentMethod), nullable=False)
    status: Mapped[PaymentStatus] = mapped_column(
        SQLEnum(PaymentStatus), nullable=False, default=PaymentStatus.PENDING
    )
    transaction_id: Mapped[str | None] = mapped_column(String(100), nullable=True)
    pg_response: Mapped[dict | None] = mapped_column(JSON, nullable=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), server_default=func.now(), nullable=False
    )
    completed_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
