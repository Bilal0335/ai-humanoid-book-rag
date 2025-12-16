"""
Main API router for the RAG Chatbot Backend API
Includes all versioned API routes.
"""
from fastapi import APIRouter
from src.api.v1 import chat, ingestion, health


# Main API router that includes all versioned API routes
api_router = APIRouter()

# Include v1 API routes under the /api/v1 prefix
api_router.include_router(chat.router, prefix="/api/v1", tags=["chat-v1"])
api_router.include_router(ingestion.router, prefix="/api/v1", tags=["ingestion-v1"])
api_router.include_router(health.router, prefix="/api/v1", tags=["health-v1"])


# Include other versioned routes as they become available
# api_router.include_router(v2.router, prefix="/api/v2", tags=["chat-v2"])