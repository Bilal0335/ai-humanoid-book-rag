from fastapi import FastAPI
from src.api import router as api_router
from src.core.config import settings
from src.core.logging import setup_logging

# Setup logging
setup_logging()

app = FastAPI(
    title="Integrated RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot embedded in technical books",
    version="0.1.0",
)

# Include API routes
app.include_router(api_router, prefix="/api")

@app.get("/")
def read_root():
    return {"message": "Integrated RAG Chatbot API is running"}

@app.get("/health")
def health_check():
    # This is just a basic health check - in a real implementation,
    # we would check actual dependencies like database, vector store, and AI service
    return {"status": "healthy", "timestamp": "2025-01-15T10:30:00Z"}