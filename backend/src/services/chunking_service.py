"""
Chunking service for the RAG Chatbot Backend API
Implements deterministic text chunking with sentence-aware logic and stable chunk IDs.
"""
import hashlib
import re
from typing import List, Dict, Any
from src.core.config import settings
from src.core.logging import get_logger
from src.core.exceptions import RAGException

logger = get_logger(__name__)


class ChunkingService:
    """
    Service for splitting book content into text segments with sentence-aware logic
    and deterministic chunking to ensure stable chunk IDs.
    """
    
    def __init__(self):
        self.chunk_size = settings.chunk_size_tokens  # tokens (defined in config)
        self.chunk_overlap = settings.chunk_overlap_tokens  # tokens (defined in config)
        self.max_chunk_size = settings.max_tokens_per_chunk
        self.min_chunk_size = 50  # Minimum size to avoid very small chunks
    
    def chunk_text(self, 
                   text: str, 
                   source_chapter: str, 
                   source_section: str,
                   book_id: str = "default_book") -> List[Dict[str, Any]]:
        """
        Split text into chunks with sentence awareness to prevent breaking sentences.
        
        Args:
            text: The text to be chunked
            source_chapter: The chapter where this text originates
            source_section: The specific section within the chapter
            book_id: Identifier for the book
            
        Returns:
            A list of dictionaries containing chunk information
        """
        logger.info(f"Starting to chunk text from {source_chapter} - {source_section}, total length: {len(text)}")
        
        if not text.strip():
            raise RAGException("CHUNKING_ERROR", "Cannot chunk empty text")
        
        # First, split the text into sentences
        sentences = self._split_into_sentences(text)
        
        # Then, group sentences into chunks of approximately the target size
        chunks = self._group_sentences_into_chunks(
            sentences=sentences,
            source_chapter=source_chapter,
            source_section=source_section,
            book_id=book_id
        )
        
        logger.info(f"Created {len(chunks)} chunks from text")
        return chunks
    
    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using regex patterns.
        
        Args:
            text: The text to split into sentences
            
        Returns:
            A list of sentences
        """
        # Pattern to split on sentence boundaries while preserving them
        # This pattern looks for periods, exclamation marks, or question marks followed by 
        # whitespace and a capitalized letter (indicating the start of a new sentence)
        sentence_endings = r'(?<!\w\.\w.)(?<![A-Z][a-z].)(?<=[.!?])\s+(?=[A-Z])'
        sentences = re.split(sentence_endings, text)
        
        # Clean up the sentences, removing empty strings and extra whitespace
        cleaned_sentences = [s.strip() for s in sentences if s.strip()]
        
        logger.debug(f"Split text into {len(cleaned_sentences)} sentences")
        return cleaned_sentences
    
    def _group_sentences_into_chunks(self,
                                    sentences: List[str],
                                    source_chapter: str,
                                    source_section: str,
                                    book_id: str) -> List[Dict[str, Any]]:
        """
        Group sentences into chunks of approximately the target size.

        Args:
            sentences: List of sentences to group
            source_chapter: The chapter where this text originates
            source_section: The specific section within the chapter
            book_id: Identifier for the book

        Returns:
            A list of dictionaries containing chunk information
        """
        chunks = []
        current_chunk_sentences = []
        current_chunk_size = 0
        chunk_order = 0

        i = 0
        while i < len(sentences):
            sentence = sentences[i]
            sentence_token_count = len(sentence.split())  # Simple tokenization for demo purposes

            # Check if adding this sentence would exceed the maximum chunk size
            if current_chunk_size + sentence_token_count > self.chunk_size and current_chunk_sentences:
                # Finalize the current chunk
                chunk_content = " ".join(current_chunk_sentences).strip()

                # Generate a stable ID based on source info, content, and chunk order
                chunk_id = self._generate_stable_chunk_id(
                    book_id=book_id,
                    source_chapter=source_chapter,
                    source_section=source_section,
                    content=chunk_content,
                    order=chunk_order
                )

                chunks.append({
                    "id": chunk_id,
                    "payload": {
                        "content": chunk_content,
                        "source_chapter": source_chapter,
                        "source_section": source_section,
                        "chunk_order": chunk_order
                    }
                })

                # Start new chunk, potentially with overlap
                chunk_order += 1

                # Add overlap by including some previous sentences if available
                if self.chunk_overlap > 0 and current_chunk_sentences:
                    # Select sentences from the end of the current chunk for overlap
                    sentences_for_overlap = []
                    temp_tokens = 0
                    for sent in reversed(current_chunk_sentences):
                        sent_tokens = len(sent.split())
                        if temp_tokens + sent_tokens <= self.chunk_overlap:
                            sentences_for_overlap.insert(0, sent)  # Insert at beginning to maintain order
                            temp_tokens += sent_tokens
                        else:
                            break

                    current_chunk_sentences = sentences_for_overlap
                    current_chunk_size = temp_tokens
                else:
                    current_chunk_sentences = []
                    current_chunk_size = 0

                # Process the same sentence again with the new context
            else:
                # Add the sentence to the current chunk
                current_chunk_sentences.append(sentence)
                current_chunk_size += sentence_token_count
                i += 1

        # Add the final chunk if it has content
        if current_chunk_sentences:
            chunk_content = " ".join(current_chunk_sentences).strip()

            # Generate a stable ID for the final chunk
            chunk_id = self._generate_stable_chunk_id(
                book_id=book_id,
                source_chapter=source_chapter,
                source_section=source_section,
                content=chunk_content,
                order=chunk_order
            )

            chunks.append({
                "id": chunk_id,
                "payload": {
                    "content": chunk_content,
                    "source_chapter": source_chapter,
                    "source_section": source_section,
                    "chunk_order": chunk_order
                }
            })

        logger.debug(f"Grouped sentences into {len(chunks)} final chunks")
        return chunks
    
    def _generate_stable_chunk_id(self, 
                                 book_id: str, 
                                 source_chapter: str, 
                                 source_section: str, 
                                 content: str, 
                                 order: int) -> str:
        """
        Generate a stable, deterministic chunk ID based on content and location.
        
        Args:
            book_id: Identifier for the book
            source_chapter: Chapter where the chunk comes from
            source_section: Section where the chunk comes from
            content: Content of the chunk
            order: Order of this chunk in the sequence
            
        Returns:
            A stable chunk identifier
        """
        # Create a unique but reproducible hash based on content and location
        content_hash = hashlib.sha256(content.encode()).hexdigest()[:16]
        location_identifier = f"{book_id}:{source_chapter}:{source_section}:{order:04d}"
        
        # Combine location and content hash to create a stable but unique ID
        chunk_id = f"{location_identifier}:{content_hash}"
        
        return chunk_id
    
    def validate_chunk(self, chunk: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a chunk meets the requirements.
        
        Args:
            chunk: A chunk dictionary to validate
            
        Returns:
            A dictionary containing validation results
        """
        content = chunk.get("payload", {}).get("content", "")
        source_chapter = chunk.get("payload", {}).get("source_chapter", "")
        source_section = chunk.get("payload", {}).get("source_section", "")
        chunk_id = chunk.get("id", "unknown")
        
        # Check if content is within size limits
        content_tokens = len(content.split())
        token_count_valid = self.min_chunk_size <= content_tokens <= self.max_chunk_size
        
        validation_result = {
            "id_valid": bool(chunk_id and chunk_id != "unknown"),
            "content_valid": bool(content.strip()) and token_count_valid,
            "source_chapter_valid": bool(source_chapter.strip()),
            "source_section_valid": bool(source_section.strip()),
            "token_count_valid": token_count_valid,
            "content_length": content_tokens,
            "is_valid": False
        }
        
        # Overall validation
        validation_result["is_valid"] = all([
            validation_result["id_valid"],
            validation_result["content_valid"],
            validation_result["source_chapter_valid"],
            validation_result["source_section_valid"],
            validation_result["token_count_valid"]
        ])
        
        # Log validation issues if any
        if not validation_result["is_valid"]:
            invalid_fields = [
                field for field, is_valid in validation_result.items()
                if not is_valid and field not in ["content_length", "is_valid"]
            ]
            logger.warning(f"Chunk {chunk_id} failed validation: {invalid_fields}")
        
        logger.debug(f"Chunk validation result: {validation_result}")
        return validation_result


# Global instance for use throughout the application
chunking_service = ChunkingService()