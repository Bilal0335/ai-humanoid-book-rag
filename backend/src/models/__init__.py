"""
Database models for the RAG Chatbot Backend API
"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from src.core.config import settings
from src.core.logging import get_logger

logger = get_logger(__name__)

# Initialize database engine and base
SQLALCHEMY_DATABASE_URL = settings.neon_database_url

engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    pool_pre_ping=True,  # Verify connections are healthy before using them
    pool_recycle=300,   # Recycle connections every 5 minutes
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


def get_db():
    """
    Dependency function to get database session.
    This is meant to be used with FastAPI's dependency injection system.
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Import all models to ensure they're registered with the Base
from .text_segment import BookTextSegment
from .book_section import BookSection
from .conversation_session import ConversationSession