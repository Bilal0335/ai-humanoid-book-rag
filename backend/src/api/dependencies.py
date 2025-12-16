"""
Dependency injection for API endpoints in the RAG Chatbot Backend API
"""
from fastapi import Header, HTTPException, status
from typing import Optional
from src.core.config import settings
from src.core.security import validate_api_key as security_validate_api_key


async def get_api_key_header(authorization: str = Header(None)):
    """
    Extract and validate API key from the Authorization header.

    Args:
        authorization: Authorization header value

    Returns:
        Validated API key

    Raises:
        HTTPException: If API key is missing or invalid
    """
    if not authorization:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header is required"
        )

    # Extract the API key from the header (assuming format: "Bearer <api_key>")
    try:
        scheme, api_key = authorization.split(" ", 1)
        if scheme.lower() != "bearer":
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authorization scheme must be Bearer"
            )
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authorization header format"
        )

    # Validate the API key
    if not security_validate_api_key(api_key):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key"
        )

    return api_key


async def get_current_user(api_key: str = Header(None)):
    """
    Extract current user information based on the API key.
    
    Args:
        api_key: The validated API key
        
    Returns:
        User information (currently just returns the API key for simplicity)
    """
    # In a real implementation, this would look up user information based on the API key
    # For now, just return a placeholder user
    return {"id": "placeholder_user_id", "api_key_valid": True}