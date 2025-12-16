"""
BookTextSegment model for the RAG Chatbot Backend API
Represents a segment of book content with stable ID, source reference, and text content.
"""
from sqlalchemy import Column, String, Integer, DateTime, Text
from sqlalchemy.sql import func
from typing import TYPE_CHECKING
from src.models import Base

if TYPE_CHECKING:
    from .conversation_session import ConversationSession  # For type hints only


class BookTextSegment(Base):
    """
    Model representing a segment of book content with stable ID, 
    source chapter/section reference, and the text content.
    """
    __tablename__ = "book_text_segments"

    # Fields as defined in the specification
    id = Column(String, primary_key=True, index=True)
    content = Column(Text, nullable=False)
    source_chapter = Column(String, nullable=False)
    source_section = Column(String, nullable=False)
    chunk_order = Column(Integer, nullable=False)
    
    # Reference to corresponding vector in vector store
    vector_id = Column(String, nullable=True)
    
    # Relationship fields
    # book_section_id = Column(String, ForeignKey("book_sections.id"), nullable=False)
    
    # Timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<BookTextSegment(id='{self.id}', chapter='{self.source_chapter}', section='{self.source_section}')>"