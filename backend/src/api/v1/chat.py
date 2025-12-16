from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from src.api.dependencies import get_api_key_header, get_current_user
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.core.config import settings
from src.core.logging import get_logger

logger = get_logger(__name__)
router = APIRouter()


class ChatRequest(BaseModel):
    query: str
    mode: str = "book-wide"  # Default to book-wide mode
    selected_text: Optional[str] = None  # Only used in selected-text mode
    current_location: Optional[dict] = None  # Used for context-aware mode (optional)


class ChatResponse(BaseModel):
    response: str
    sources: list
    mode_used: str
    suggested_sections: Optional[list] = None  # Added for context-aware suggestions
    timestamp: str


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    api_key: str = Depends(get_api_key_header())
):
    """
    Process user questions using book-wide retrieval, selected-text context, or context-aware responses.

    Args:
        request: Chat request with query, mode, selected text, and optional current location
        api_key: API key for authentication

    Returns:
        ChatResponse with the answer, sources, and potentially suggested sections
    """
    logger.info(f"Received chat request with mode: {request.mode}, query: {request.query[:50]}...")

    try:
        # Validate mode
        if request.mode not in ["book-wide", "selected-text", "context-aware"]:
            logger.error(f"Invalid mode: {request.mode}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": {
                        "code": "INVALID_MODE",
                        "message": f"Invalid mode: {request.mode}. Must be 'book-wide', 'selected-text', or 'context-aware'"
                    }
                }
            )

        # If mode is selected-text, validate that selected_text is provided
        if request.mode == "selected-text" and not request.selected_text:
            logger.error("Selected-text mode requires selected_text parameter")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": {
                        "code": "VALIDATION_ERROR",
                        "message": "selected_text parameter is required in selected-text mode"
                    }
                }
            )

        # Process based on mode
        if request.mode == "book-wide":
            # Perform vector-based retrieval based on the query
            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )

            # Generate response using the retrieved context
            result = generation_service.generate_answer_with_context_awareness(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )
        elif request.mode == "selected-text":
            # Bypass vector retrieval and use only the selected text as context
            # The retrieval service should return empty chunks in selected-text mode
            context_chunks_from_retrieval = retrieval_service.retrieve_relevant_chunks(
                query=request.query,  # This is essentially ignored in selected-text mode
                mode=request.mode
            )

            # Create context from the selected text provided by the user
            context_chunks = [{
                "id": "selected-text",
                "payload": {
                    "content": request.selected_text,
                    "source_chapter": "Selected Text",
                    "source_section": "Selected Text"
                }
            }]

            # Generate response using only the selected text as context
            result = generation_service.generate_answer_with_context_awareness(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )
        else:  # context-aware mode
            # Use the user's current location to provide more relevant responses
            context_chunks = retrieval_service.retrieve_in_context_aware_manner(
                query=request.query,
                current_chapter=request.current_location.get('chapter') if request.current_location else None,
                current_section=request.current_location.get('section') if request.current_location else None,
                nearby_sections=request.current_location.get('nearby_sections') if request.current_location else None
            )

            # Find related sections that might be useful to the user
            related_sections = []
            if request.current_location:
                concept_to_find = request.query.split()[0] if request.query.split() else ""  # Simplified approach to extract topic
                if concept_to_find:
                    related_sections = retrieval_service.find_related_sections(
                        concept=concept_to_find,
                        current_location=request.current_location
                    )

            # Generate context-aware response
            result = generation_service.generate_answer_with_context_awareness(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode,
                current_location=request.current_location,
                related_sections=related_sections
            )

        # Log the response
        logger.info(f"Generated response for query: {request.query[:50]}...")

        # Return the formatted response
        return ChatResponse(
            response=result["response"],
            sources=result["sources"],
            mode_used=result["mode_used"],
            suggested_sections=result.get("suggested_sections", []),
            timestamp=result["timestamp"]
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": {
                    "code": "INTERNAL_ERROR",
                    "message": "An internal error occurred while processing the request"
                }
            }
        )


@router.get("/health")
async def health_check():
    """
    Check the health status of the service and its dependencies.

    Returns:
        Health status of the service
    """
    import datetime

    # TODO: Add actual health checks for dependencies like database, vector store, and AI service
    health_status = {
        "status": "healthy",
        "checks": {
            "database": "not_implemented_yet",  # Will be implemented later
            "vector_store": "not_implemented_yet",  # Will be implemented later
            "ai_service": "not_implemented_yet",  # Will be implemented later
            "timestamp": datetime.datetime.utcnow().isoformat() + "Z"
        }
    }

    return health_status