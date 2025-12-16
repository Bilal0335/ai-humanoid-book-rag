"""
ConversationSession model for the RAG Chatbot Backend API
Represents a conversation session with user interactions and context tracking.
"""
from sqlalchemy import Column, String, Integer, DateTime, Text, JSON
from sqlalchemy.sql import func
from src.models import Base
import uuid


class ConversationSession(Base):
    """
    Model containing a series of user interactions with the chatbot,
    stored securely with user context and reading position tracking.
    """
    __tablename__ = "conversation_sessions"

    # Fields as defined in the specification
    session_id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()), index=True)
    user_id = Column(String, nullable=True)  # Identifier for the user (if available)
    mode = Column(String, nullable=False)  # Either "book-wide", "selected-text", or "context-aware"

    # Reading position tracking (for context-aware features)
    current_book_id = Column(String, nullable=True)  # ID of the book being read
    current_chapter_id = Column(String, nullable=True)  # Current chapter ID
    current_section_id = Column(String, nullable=True)  # Current section ID
    current_page = Column(Integer, nullable=True)  # Current page number in the book
    last_read_timestamp = Column(DateTime(timezone=True), nullable=True)  # Last interaction timestamp

    # Timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    expires_at = Column(DateTime(timezone=True))  # Expiration timestamp for session cleanup

    # Store messages as JSON as it's a complex nested structure
    messages = Column(JSON, nullable=False, default=list)  # List of message objects

    def __repr__(self):
        return f"<ConversationSession(id='{self.session_id}', user='{self.user_id}', mode='{self.mode}')>"

    def update_reading_position(self, book_id: str = None, chapter_id: str = None, 
                               section_id: str = None, page: int = None):
        """
        Updates the reading position in the session.
        
        Args:
            book_id: ID of the book being read
            chapter_id: Current chapter ID 
            section_id: Current section ID
            page: Current page number
        """
        if book_id:
            self.current_book_id = book_id
        if chapter_id:
            self.current_chapter_id = chapter_id
        if section_id:
            self.current_section_id = section_id
        if page is not None:
            self.current_page = page
            
        self.last_read_timestamp = func.now()

    def get_current_reading_position(self) -> dict:
        """
        Gets the current reading position.
        
        Returns:
            Dictionary with current reading position information
        """
        return {
            "book_id": self.current_book_id,
            "chapter_id": self.current_chapter_id,
            "section_id": self.current_section_id,
            "page": self.current_page,
            "last_read_at": self.last_read_timestamp
        }