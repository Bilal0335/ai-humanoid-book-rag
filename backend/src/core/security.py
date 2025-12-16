"""
Security utilities and credential management for the RAG Chatbot Backend API
"""
import secrets
from passlib.context import CryptContext
from typing import Optional
import os


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.

    Args:
        plain_password: Plain text password to verify
        hashed_password: Previously hashed password to compare against

    Returns:
        True if passwords match, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """
    Generate a hash for a plain password.

    Args:
        password: Plain text password to hash

    Returns:
        Hashed password string
    """
    return pwd_context.hash(password)


def generate_secure_token(length: int = 32) -> str:
    """
    Generate a cryptographically secure random token.

    Args:
        length: Length of the token in bytes (default 32)

    Returns:
        Hex-encoded random token
    """
    return secrets.token_hex(length)


def validate_api_key(provided_api_key: str) -> bool:
    """
    Validate an API key against the configured API key.

    Args:
        provided_api_key: API key provided by the user

    Returns:
        True if the API key is valid, False otherwise
    """
    expected_api_key = os.getenv("API_KEY")
    if not expected_api_key:
        # If no expected API key is configured, validation fails
        return False

    return secrets.compare_digest(provided_api_key, expected_api_key)


def sanitize_input(user_input: str) -> str:
    """
    Sanitize user input to prevent injection attacks.

    Args:
        user_input: Raw user input

    Returns:
        Sanitized input string
    """
    if not user_input:
        return user_input

    # Remove null bytes and other potentially harmful characters
    sanitized = user_input.replace('\x00', '')  # Remove null bytes

    # Additional sanitization could be implemented here based on specific needs

    return sanitized