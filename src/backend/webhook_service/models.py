from sqlalchemy import (
    Boolean,
    Column,
    DateTime,
    Float,
    Integer,
    String,
    func,
    text,
)
from sqlalchemy.orm import declarative_base


Base = declarative_base()


class WebhookWeightment(Base):
    __tablename__ = "webhook_weightments"

    id = Column(Integer, primary_key=True, index=True)
    event_id = Column(String, index=True, nullable=False)
    sent_utc = Column(String, nullable=True)
    user_id = Column(String, nullable=True)
    site_id = Column(String, index=True, nullable=True)
    batch_id = Column(String, index=True, nullable=True)
    batch_number = Column(String, index=True, nullable=True)
    work_order_id = Column(String, index=True, nullable=True)
    batch_target_quantity = Column(Float, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    target_weight_kg = Column(Float, nullable=True)
    actual_weight_kg = Column(Float, nullable=True)
    weightment_completed = Column(Boolean, nullable=False, default=False)
    mes_timeseries_sent = Column(
        Boolean, nullable=False, default=False, server_default=text("false")
    )
    batch_auto_run_enabled = Column(
        Boolean, nullable=False, default=False, server_default=text("false")
    )
    start_utc = Column(String, nullable=True)
    end_utc = Column(String, nullable=True)
    energy_kwh = Column(Integer, nullable=True)
    lot_code = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class StockLocationAllocation(Base):
    __tablename__ = "stock_location_allocations"

    id = Column(Integer, primary_key=True, index=True)
    event_id = Column(String, index=True, nullable=False)
    context_id = Column(String, nullable=True)
    site_id = Column(String, index=True, nullable=True)
    created_utc = Column(String, nullable=True)
    stock_item_location_id = Column(Integer, index=True, nullable=True)
    stock_item_location_code = Column(String, index=True, nullable=True)
    stock_item_id = Column(String, index=True, nullable=True)
    stock_item_code = Column(String, nullable=True)
    stock_item_name = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
