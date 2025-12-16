"""
Database connection and setup for the RAG Chatbot Backend API
Uses SQLAlchemy for PostgreSQL connection with Neon Serverless.
"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import StaticPool
from typing import Generator
from src.core.config import settings
from src.core.logging import get_logger

logger = get_logger(__name__)

# Create the database engine
if settings.neon_database_url:
    engine = create_engine(
        settings.neon_database_url,
        pool_pre_ping=True,  # Verify connections are healthy before using them
        pool_recycle=300,   # Recycle connections every 5 minutes
    )
else:
    # For development without database credentials, use in-memory SQLite
    logger.warning("No database URL provided, using in-memory SQLite for development")
    engine = create_engine(
        "sqlite:///:memory:",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool
    )

# Create a configured "SessionLocal" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create a Base class for declarative models
Base = declarative_base()


def get_db() -> Generator[Session, None, None]:
    """
    Dependency to get database session for FastAPI endpoints.
    
    Yields:
        Database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def init_db():
    """
    Initialize the database by creating all tables.
    This should be called when the application starts.
    """
    logger.info("Initializing database and creating tables if they don't exist")
    Base.metadata.create_all(bind=engine)
    logger.info("Database initialization completed")


def get_db_health() -> bool:
    """
    Check if the database is accessible.
    
    Returns:
        True if database is accessible, False otherwise
    """
    try:
        with SessionLocal() as db:
            # Perform a simple query to check connectivity
            db.execute("SELECT 1")
        logger.debug("Database health check passed")
        return True
    except Exception as e:
        logger.error(f"Database health check failed: {str(e)}")
        return False


# Initialize the database when this module is imported
init_db()