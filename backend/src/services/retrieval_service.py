from typing import List, Dict, Any, Optional
from src.core.vector_store import vector_store
from src.core.embedding_service import embedding_service
from src.core.config import settings
from src.core.logging import get_logger
from src.core.exceptions import RAGException

logger = get_logger(__name__)


class RetrievalService:
    """
    Service for performing vector search and retrieval operations.
    """

    def __init__(self):
        self.top_k = settings.top_k_retrieval
        self.vector_store = vector_store
        self.embedding_service = embedding_service

    def retrieve_relevant_chunks(self,
                                query: str,
                                top_k: Optional[int] = None,
                                filters: Optional[Dict[str, Any]] = None,
                                mode: str = "book-wide") -> List[Dict[str, Any]]:
        """
        Retrieve relevant text chunks based on a query.

        Args:
            query: The query text to search for
            top_k: Number of top results to return (defaults to config value)
            filters: Optional filters to apply to the search
            mode: Either "book-wide" for vector search or "selected-text" to bypass search

        Returns:
            A list of relevant chunks with their metadata
        """
        if not query.strip():
            raise RAGException("RETRIEVAL_ERROR", "Query cannot be empty")

        try:
            logger.debug(f"Starting retrieval for query in mode: {mode}, query: {query[:50]}...")

            # If in selected-text mode, bypass vector search and return empty results
            # The context will be provided separately from the selected text
            if mode == "selected-text":
                logger.info("Bypassing vector search in selected-text mode")
                return []

            # Generate embedding for the query (for book-wide mode)
            query_embedding = self.embedding_service.generate_embedding(query)

            # Perform vector search
            top_k = top_k or self.top_k
            results = self.vector_store.search_similar(
                query_embedding=query_embedding,
                top_k=top_k,
                filters=filters
            )

            logger.info(f"Retrieved {len(results)} relevant chunks for query in book-wide mode")
            return results

        except RAGException:
            # Re-raise RAG exceptions as they're already properly formatted
            raise
        except Exception as e:
            logger.error(f"Unexpected error in retrieve_relevant_chunks: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to retrieve relevant chunks: {str(e)}")

    def retrieve_by_chapter_section(self,
                                   chapter: str,
                                   section: str,
                                   top_k: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Retrieve chunks specifically from a given chapter and section.

        Args:
            chapter: The chapter to search in
            section: The section to search in
            top_k: Number of top results to return (defaults to config value)

        Returns:
            A list of chunks from the specified chapter and section
        """
        try:
            logger.debug(f"Retrieving chunks for chapter: {chapter}, section: {section}")

            # Create filters for the specific chapter and section
            filters = {
                "source_chapter": chapter,
                "source_section": section
            }

            # For now, we'll use a simple embedding of the query to pass to search_similar
            # In a real implementation, this would search with a null/neutral query
            dummy_query_embedding = self.embedding_service.generate_embedding(f"Retrieve content from {chapter} section {section}")

            # Perform vector search with filters
            top_k = top_k or self.top_k
            results = self.vector_store.search_similar(
                query_embedding=dummy_query_embedding,
                top_k=top_k,
                filters=filters
            )

            logger.info(f"Retrieved {len(results)} chunks from chapter {chapter}, section {section}")
            return results

        except RAGException:
            # Re-raise RAG exceptions as they're already properly formatted
            raise
        except Exception as e:
            logger.error(f"Unexpected error in retrieve_by_chapter_section: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to retrieve chunks by chapter/section: {str(e)}")

    def retrieve_in_context_aware_manner(self,
                                         query: str,
                                         current_chapter: Optional[str] = None,
                                         current_section: Optional[str] = None,
                                         nearby_sections: Optional[List[str]] = None,
                                         top_k: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Retrieve chunks considering the user's current reading position and context.

        Args:
            query: The query text to search for
            current_chapter: The chapter the user is currently reading
            current_section: The specific section the user is currently reading
            nearby_sections: List of related sections nearby to include in search
            top_k: Number of top results to return (defaults to config value)

        Returns:
            A list of relevant chunks with positional context awareness
        """
        try:
            logger.debug(f"Starting context-aware retrieval for query: {query[:50]}..., current location: {current_chapter} - {current_section}")

            # Create embedding for the query
            query_embedding = self.embedding_service.generate_embedding(query)

            # Create filters to prioritize content from current context
            filters = {}
            if current_chapter and current_section:
                # Boost results from the current section
                filters = {
                    "source_chapter": current_chapter,
                    "source_section": current_section
                }
            elif current_chapter:
                # If only chapter is specified, search within that chapter
                filters = {
                    "source_chapter": current_chapter
                }

            top_k = top_k or self.top_k

            # Perform vector search with context-based filters
            results = self.vector_store.search_similar(
                query_embedding=query_embedding,
                top_k=top_k,
                filters=filters
            )

            # If no results from current context and nearby sections are specified,
            # broaden the search to include nearby sections
            if len(results) == 0 and nearby_sections:
                logger.debug(f"No results from current context, expanding to nearby sections: {nearby_sections}")

                # Search in nearby sections as well
                for section in nearby_sections:
                    # This is a simplified approach - in practice you'd want to search in each section
                    # and potentially weight results based on proximity to current reading position
                    nearby_filters = {
                        "source_section": section
                    }

                    nearby_results = self.vector_store.search_similar(
                        query_embedding=query_embedding,
                        top_k=top_k // 2,  # Use fewer results from nearby sections
                        filters=nearby_filters
                    )

                    # Add nearby results to main results, avoiding duplicates
                    for nr in nearby_results:
                        if not any(r.get('id') == nr.get('id') for r in results):
                            results.append(nr)

            logger.info(f"Context-aware retrieval returned {len(results)} chunks from current/nearby sections")
            return results

        except RAGException:
            # Re-raise RAG exceptions as they're already properly formatted
            raise
        except Exception as e:
            logger.error(f"Unexpected error in retrieve_in_context_aware_manner: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to retrieve contextually relevant chunks: {str(e)}")

    def find_related_sections(self,
                              concept: str,
                              current_location: Optional[Dict[str, str]] = None,
                              max_results: int = 5) -> List[Dict[str, Any]]:
        """
        Find sections related to a specific concept, optionally considering current reading location.

        Args:
            concept: The concept to search for
            current_location: Dictionary with current reading location (chapter, section, etc.)
            max_results: Maximum number of related sections to return

        Returns:
            A list of related sections
        """
        try:
            logger.debug(f"Finding related sections for concept: {concept}, current location: {current_location}")

            # Create embedding for the concept
            concept_embedding = self.embedding_service.generate_embedding(concept)

            # Create filters based on current location if provided
            filters = None
            if current_location:
                # Exclude current location to find truly related but different sections
                filters = {
                    "excluded_sections": [current_location.get("section_id", "")],
                    "excluded_chapters": [current_location.get("chapter_id", "")]
                }
            else:
                filters = {}

            # Perform vector search for the concept
            results = self.vector_store.search_similar(
                query_embedding=concept_embedding,
                top_k=max_results,
                filters=filters
            )

            logger.info(f"Found {len(results)} related sections for concept: {concept}")
            return results

        except RAGException:
            # Re-raise RAG exceptions as they're already properly formatted
            raise
        except Exception as e:
            logger.error(f"Unexpected error in find_related_sections: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to find related sections: {str(e)}")

    def retrieve_by_ids(self, ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve specific chunks by their IDs.

        Args:
            ids: List of chunk IDs to retrieve

        Returns:
            A list of chunks corresponding to the specified IDs
        """
        try:
            logger.debug(f"Retrieving chunks by IDs: {ids}")

            # This would require a different approach since our vector store doesn't support direct ID lookup
            # For now, we'll just return an empty list and implement this properly if needed
            # This could be implemented by storing chunk IDs in metadata and searching by that
            results = []

            # In a real implementation, we would retrieve these from the vector store and/or database
            logger.warning("Direct ID retrieval not fully implemented in this version")

            return results
        except Exception as e:
            logger.error(f"Unexpected error in retrieve_by_ids: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to retrieve chunks by IDs: {str(e)}")

    def get_all_chunks_for_section(self, section_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks for a specific section.

        Args:
            section_id: The section ID to retrieve chunks for

        Returns:
            A list of all chunks for the specified section
        """
        try:
            logger.debug(f"Retrieving all chunks for section ID: {section_id}")

            # This would require implementation in the vector store to support
            # metadata-based filtering for all items in a section
            results = []

            logger.info(f"Retrieved {len(results)} chunks for section {section_id}")
            return results
        except Exception as e:
            logger.error(f"Unexpected error in get_all_chunks_for_section: {str(e)}")
            raise RAGException("RETRIEVAL_ERROR", f"Failed to retrieve all chunks for section: {str(e)}")


# Global instance for use throughout the application
retrieval_service = RetrievalService()