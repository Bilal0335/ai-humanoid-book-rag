"""
Constants module for the RAG Chatbot Backend API
"""

# Default values
DEFAULT_MAX_TOKENS = 500
DEFAULT_TEMPERATURE = 0.3
DEFAULT_TOP_K_RETRIEVAL = 5
DEFAULT_CHUNK_SIZE = 350
DEFAULT_CHUNK_OVERLAP = 50
DEFAULT_MAX_SEGMENT_TOKENS = 400

# Response messages
INSUFFICIENT_CONTEXT_MESSAGE = "The answer is not available in the selected text."
FALLBACK_UNANSWERABLE_MESSAGE = "I cannot answer that question as it's not covered in the book content."

# API Response codes
INVALID_MODE_ERROR = "INVALID_MODE"
VALIDATION_ERROR = "VALIDATION_ERROR"
INSUFFICIENT_CONTEXT_ERROR = "INSUFFICIENT_CONTEXT"
INTERNAL_ERROR = "INTERNAL_ERROR"

# Validation thresholds
CONTEXT_ALIGNMENT_THRESHOLD = 0.7  # 70% alignment minimum for valid responses
RESPONSE_COVERAGE_THRESHOLD = 0.6  # 60% of response should be sourced from context