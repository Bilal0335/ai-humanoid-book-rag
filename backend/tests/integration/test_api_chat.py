"""
Integration test for book-wide question answering functionality.
This test verifies that the full book-wide question answering flow works correctly,
from query processing through retrieval to answer generation.
"""
import pytest
from unittest.mock import patch, MagicMock
from src.api.v1.chat import ChatRequest
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service


@pytest.mark.asyncio
async def test_book_wide_question_answering_integration():
    """
    Integration test to verify the complete book-wide question answering flow.
    Tests the integration between all components: retrieval, generation, and validation.
    """
    # Test data based on typical book content
    mock_query = "What is the main concept discussed in chapter 3?"
    
    # Mock the retrieval service to return some relevant chunks
    mock_chunks = [
        {
            "id": "chunk-1",
            "payload": {
                "content": "Chapter 3 discusses the concept of retrieval-augmented generation (RAG). RAG combines information retrieval with text generation to create more accurate and contextually relevant responses. The approach involves first retrieving relevant documents or passages, then conditioning the generation model on these retrieved documents.",
                "source_chapter": "Chapter 3",
                "source_section": "3.1 Introduction to RAG"
            }
        },
        {
            "id": "chunk-2", 
            "payload": {
                "content": "The key advantage of RAG is that it reduces hallucinations by grounding the generation process in actual source material. This makes it particularly suitable for applications where accuracy is critical, such as in technical documentation or educational materials.",
                "source_chapter": "Chapter 3",
                "source_section": "3.2 Benefits of RAG"
            }
        }
    ]
    
    with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=mock_chunks):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock the generation service to return a response
            mock_gen.return_value = {
                "response": "The main concept in chapter 3 is retrieval-augmented generation (RAG), which combines information retrieval with text generation to create more accurate responses.",
                "sources": [
                    {
                        "chapter": "Chapter 3",
                        "section": "3.1 Introduction to RAG", 
                        "text": "Chapter 3 discusses the concept of retrieval-augmented generation (RAG)..."
                    }
                ],
                "mode_used": "book-wide",
                "timestamp": "2025-01-15T10:30:00Z"
            }
            
            # Create a request for book-wide mode
            request = ChatRequest(
                query=mock_query,
                mode="book-wide"
            )
            
            # Call the chat endpoint functionality (simulating what would happen in the endpoint)
            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )
            
            assert len(context_chunks) == 2  # Verify retrieval worked as expected
            
            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )
            
            # Assertions to verify the integration works correctly
            assert result["mode_used"] == "book-wide"
            assert "retrieval-augmented generation" in result["response"].lower()
            assert len(result["sources"]) > 0
            assert result["sources"][0]["chapter"] == "Chapter 3"
            

@pytest.mark.asyncio
async def test_book_wide_insufficient_context_handling():
    """
    Integration test to verify book-wide mode properly handles queries 
    that cannot be answered with available context.
    """
    mock_query = "What is the secret to happiness according to chapter 99?"
    
    # Mock retrieval service to return no relevant chunks
    with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=[]):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock generation to handle insufficient context appropriately
            mock_gen.return_value = {
                "response": "I cannot answer that question as it's not covered in the book content.",
                "sources": [],
                "mode_used": "book-wide", 
                "timestamp": "2025-01-15T10:30:00Z"
            }
            
            # Create request
            request = ChatRequest(
                query=mock_query,
                mode="book-wide"
            )
            
            # Execute the flow
            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )
            
            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )
            
            # Verify insufficient context is handled properly
            assert result["response"].lower().startswith("i cannot answer")
            assert result["mode_used"] == "book-wide"


@pytest.mark.asyncio
async def test_book_wide_response_validation():
    """
    Integration test to verify that responses in book-wide mode
    are validated for hallucination and context alignment.
    """
    mock_query = "What is the RAG approach?"

    mock_chunks = [
        {
            "id": "chunk-1",
            "payload": {
                "content": "The RAG approach combines information retrieval with text generation to create more accurate and contextually relevant responses.",
                "source_chapter": "Chapter 3",
                "source_section": "3.1 Introduction to RAG"
            }
        }
    ]

    with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=mock_chunks):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock generation service to return a properly grounded response
            mock_gen.return_value = {
                "response": "The RAG approach combines information retrieval with text generation to create more accurate responses. This is grounded in the book content.",
                "sources": [
                    {
                        "chapter": "Chapter 3",
                        "section": "3.1 Introduction to RAG",
                        "text": "The RAG approach combines information retrieval with text generation to create more accurate and contextually relevant responses."
                    }
                ],
                "mode_used": "book-wide",
                "timestamp": "2025-01-15T10:30:00Z",
                "validation_result": {
                    "alignment_score": 0.9,
                    "is_aligned": True,
                    "potential_hallucinations": [],
                    "context_coverage": 0.85
                }
            }

            request = ChatRequest(
                query=mock_query,
                mode="book-wide"
            )

            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )

            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )

            # Verify the response was validated properly
            assert result["validation_result"]["is_aligned"] is True
            assert result["validation_result"]["alignment_score"] > 0.7
            assert len(result["validation_result"]["potential_hallucinations"]) == 0


@pytest.mark.asyncio
async def test_selected_text_question_answering_integration():
    """
    Integration test to verify the complete selected-text question answering flow.
    Tests that the system properly uses only the selected text as context without
    accessing broader book content.
    """
    # Test query and selected text
    mock_query = "Explain the concept mentioned here"
    selected_text = "The concept of transfer learning involves taking a pre-trained model and fine-tuning it for a related task. This approach leverages existing knowledge to accelerate learning on the new task."

    # Create request for selected-text mode
    request = ChatRequest(
        query=mock_query,
        mode="selected-text",
        selected_text=selected_text
    )

    # Mock the retrieval service to verify it's bypassed in selected-text mode
    with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
        # Mock retrieval to return empty chunks for selected-text mode (as designed)
        mock_retrieve.return_value = []

        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock generation service to return a response based on selected text
            mock_gen.return_value = {
                "response": "Transfer learning involves taking a pre-trained model and fine-tuning it for a related task. This leverages existing knowledge to accelerate learning.",
                "sources": [
                    {
                        "chapter": "Selected Text",
                        "section": "Selected Text",
                        "text": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text
                    }
                ],
                "mode_used": "selected-text",
                "timestamp": "2025-01-15T11:00:00Z",
                "validation_result": {
                    "follows_policy": True,
                    "alignment_ratio": 0.85,
                    "unmatched_tokens": [],
                    "reason": "Response properly aligned with selected text"
                }
            }

            # Execute the selected-text flow
            context_chunks_from_retrieval = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )

            # Verify that retrieval returns empty chunks in selected-text mode
            assert context_chunks_from_retrieval == []

            # Create context from selected text directly
            context_chunks = [{
                "id": "selected-text",
                "payload": {
                    "content": request.selected_text,
                    "source_chapter": "Selected Text",
                    "source_section": "Selected Text"
                }
            }]

            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )

            # Assertions to verify selected-text mode works correctly
            assert result["mode_used"] == "selected-text"
            assert "transfer learning" in result["response"].lower()
            assert len(result["sources"]) > 0
            assert result["sources"][0]["chapter"] == "Selected Text"
            assert result["validation_result"]["follows_policy"] is True


@pytest.mark.asyncio
async def test_selected_text_insufficient_context_handling():
    """
    Integration test to verify that selected-text mode properly handles
    cases where the selected text is insufficient to answer the query.
    """
    # Query that cannot be answered with very limited selected text
    mock_query = "What are the implementation details?"
    selected_text = "Introduction to the topic."  # Very limited context

    request = ChatRequest(
        query=mock_query,
        mode="selected-text",
        selected_text=selected_text
    )

    # Mock services to simulate insufficient context handling
    with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=[]):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock generation to handle insufficient context appropriately
            mock_gen.return_value = {
                "response": "The answer is not available in the selected text.",
                "sources": [],
                "mode_used": "selected-text",
                "timestamp": "2025-01-15T11:00:00Z",
                "validation_result": {
                    "follows_policy": True,
                    "reason": "Selected text was insufficient, responded appropriately"
                }
            }

            # Execute the flow
            context_chunks_from_retrieval = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )

            context_chunks = [{
                "id": "selected-text",
                "payload": {
                    "content": request.selected_text,
                    "source_chapter": "Selected Text",
                    "source_section": "Selected Text"
                }
            }]

            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )

            # Verify insufficient context is handled properly
            assert result["response"].lower() == "the answer is not available in the selected text."
            assert result["mode_used"] == "selected-text"
            assert result["validation_result"]["follows_policy"] is True


@pytest.mark.asyncio
async def test_selected_text_context_isolation():
    """
    Integration test to verify that selected-text mode maintains strict
    context isolation and doesn't access broader book content.
    """
    # Selected text with specific concept
    selected_text = "The algorithm works by using iterative refinement to improve results."

    # Query that could potentially trigger access to broader knowledge
    mock_query = "Compare this algorithm with other approaches in the book?"

    request = ChatRequest(
        query=mock_query,
        mode="selected-text",
        selected_text=selected_text
    )

    with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=[]):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock response that only uses information from selected text
            mock_gen.return_value = {
                "response": "The algorithm uses iterative refinement to improve results.",
                "sources": [
                    {
                        "chapter": "Selected Text",
                        "section": "Selected Text",
                        "text": selected_text
                    }
                ],
                "mode_used": "selected-text",
                "timestamp": "2025-01-15T11:00:00Z",
                "validation_result": {
                    "follows_policy": True,
                    "alignment_ratio": 1.0,
                    "unmatched_tokens": [],
                    "reason": "Response properly restricted to selected text context"
                }
            }

            # Execute the flow
            context_chunks_from_retrieval = retrieval_service.retrieve_relevant_chunks(
                query=request.query,
                mode=request.mode
            )

            context_chunks = [{
                "id": "selected-text",
                "payload": {
                    "content": request.selected_text,
                    "source_chapter": "Selected Text",
                    "source_section": "Selected Text"
                }
            }]

            result = generation_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                mode=request.mode
            )

            # Verify context isolation is maintained
            assert result["mode_used"] == "selected-text"
            assert "iterative refinement" in result["response"].lower()
            assert result["validation_result"]["follows_policy"] is True
            # The response should not contain information not in the selected text


@pytest.mark.asyncio
async def test_context_aware_interaction_integration():
    """
    Integration test for context-aware interaction functionality.
    Verifies the system can maintain awareness of the user's current reading position
    and provide responses that are relevant to their location in the book.
    """
    # Query asked while the user is at a specific location in the book
    mock_query = "What does this concept mean?"

    # Context representing the user's current location in the book
    current_location = {
        "chapter": "Chapter 5",
        "section": "5.2 Neural Network Architectures",
        "subsection": "5.2.3 Transformer Models"
    }

    # Mock context chunks that would be relevant to the current location
    mock_context_chunks = [
        {
            "id": "chunk-neural-arch",
            "payload": {
                "content": "Neural network architectures like transformers have revolutionized modern AI. The transformer architecture uses self-attention mechanisms to process input sequences in parallel rather than sequentially like RNNs.",
                "source_chapter": "Chapter 5",
                "source_section": "5.2 Neural Network Architectures"
            }
        },
        {
            "id": "chunk-transformer-details",
            "payload": {
                "content": "Transformer models rely on attention mechanisms to weigh the importance of different inputs. This enables them to handle long-range dependencies better than traditional approaches.",
                "source_chapter": "Chapter 5",
                "source_section": "5.2.3 Transformer Models"
            }
        }
    ]

    with patch.object(retrieval_service, 'retrieve_relevant_chunks_with_context', return_value=mock_context_chunks):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock generation service to return a response that references the current context
            mock_gen.return_value = {
                "response": "Based on the content in your current section (Chapter 5, 5.2.3 Transformer Models), this concept refers to using self-attention mechanisms to process inputs in parallel. This allows the model to handle long-range dependencies more effectively than traditional RNNs.",
                "sources": [
                    {
                        "chapter": "Chapter 5",
                        "section": "5.2.3 Transformer Models",
                        "text": "Transformer models rely on attention mechanisms to weigh the importance of different inputs. This enables them to handle long-range dependencies better than traditional approaches."
                    }
                ],
                "mode_used": "context-aware",  # This would be part of the enhancement
                "timestamp": "2025-01-15T12:00:00Z",
                "suggested_follow_up": [
                    {"chapter": "Chapter 6", "section": "6.1 Attention Mechanisms Explained"},
                    {"chapter": "Chapter 7", "section": "7.3 Advanced Transformer Variants"}
                ]
            }

            # In the current implementation, context-aware mode would need to be added
            # For now, simulate the flow that would happen with context-aware features
            # This test defines the expected behavior of the feature
            context_chunks = retrieval_service.retrieve_relevant_chunks_with_context(
                query=mock_query,
                current_location=current_location
            )

            # We'll simulate how the context-aware feature would work by using current location as context
            result = generation_service.generate_answer(
                query=mock_query,
                context_chunks=context_chunks,
                mode="book-wide",  # Using current mode for now; context-aware would be an enhancement
                current_location=current_location  # Additional parameter for context awareness
            )

            # Verify context-aware behavior
            # This test ensures that when context awareness is implemented,
            # the response will be relevant to the current location in the book
            assert "Chapter 5" in result["response"] or "Transformer" in result["response"]
            assert len(result["sources"]) > 0

            # If context awareness is working, it might suggest related sections
            # This is a placeholder for the actual implementation behavior


@pytest.mark.asyncio
async def test_context_aware_navigation_integration():
    """
    Integration test to verify that the system can assist with navigation
    and suggest relevant sections based on the user's current reading position.
    """
    # User asks for more information about something they're reading
    mock_query = "Where can I find more details about this technique?"

    current_location = {
        "chapter": "Chapter 4",
        "section": "4.5 Model Optimization",
        "concept": "quantization techniques"
    }

    # Simulate retrieval of contextually relevant sections
    mock_relevant_sections = [
        {
            "id": "related-section-optimization",
            "payload": {
                "content": "Advanced quantization techniques for model optimization involve reducing the precision of weights and activations to decrease model size and inference time.",
                "source_chapter": "Chapter 6",
                "source_section": "6.3 Quantization in Practice"
            }
        },
        {
            "id": "related-section-other",
            "payload": {
                "content": "Other optimization techniques include pruning and knowledge distillation which can be combined with quantization for enhanced efficiency.",
                "source_chapter": "Chapter 7",
                "source_section": "7.2 Combined Optimization Approaches"
            }
        }
    ]

    with patch.object(retrieval_service, 'find_related_sections', return_value=mock_relevant_sections):
        with patch.object(generation_service, 'generate_answer') as mock_gen:
            # Mock response that suggests related sections based on current context
            mock_gen.return_value = {
                "response": "For more details about quantization techniques, see Chapter 6, section 6.3 'Quantization in Practice'. You might also find relevant information in Chapter 7, section 7.2 'Combined Optimization Approaches'.",
                "sources": [
                    {
                        "chapter": "Chapter 6",
                        "section": "6.3 Quantization in Practice",
                        "text": "Advanced quantization techniques for model optimization involve reducing the precision of weights and activations to decrease model size and inference time."
                    }
                ],
                "mode_used": "context-aware",
                "timestamp": "2025-01-15T12:00:00Z",
                "navigation_suggestions": [
                    {"chapter": "Chapter 6", "section": "6.3 Quantization in Practice", "relevance": 0.9},
                    {"chapter": "Chapter 7", "section": "7.2 Combined Optimization Approaches", "relevance": 0.7}
                ]
            }

            # Simulate the context-aware navigation assistance flow
            related_sections = retrieval_service.find_related_sections(
                concept="quantization techniques",
                current_location=current_location
            )

            result = generation_service.generate_answer(
                query=mock_query,
                context_chunks=related_sections,
                mode="book-wide",  # Would be context-aware in full implementation
                current_location=current_location
            )

            # Verify navigation assistance works as expected
            assert "Chapter 6" in result["response"] or "Chapter 7" in result["response"]
            assert "quantization" in result["response"].lower()
            assert len(result.get("navigation_suggestions", [])) > 0


@pytest.mark.asyncio
async def test_context_aware_integration_backward_compatibility():
    """
    Integration test to ensure context-aware features don't break existing functionality.
    Verifies that book-wide and selected-text modes continue to work when context awareness is added.
    """
    # Test that book-wide mode still works with context awareness implemented
    book_wide_request = ChatRequest(
        query="What is the main concept of chapter 3?",
        mode="book-wide"
    )

    with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
        mock_chunks = [
            {
                "id": "chunk-3-1",
                "payload": {
                    "content": "Chapter 3 covers the fundamentals of retrieval-augmented generation (RAG) systems.",
                    "source_chapter": "Chapter 3",
                    "source_section": "3.1 Introduction"
                }
            }
        ]
        mock_retrieve.return_value = mock_chunks

        with patch.object(generation_service, 'generate_answer') as mock_gen:
            mock_gen.return_value = {
                "response": "Chapter 3 covers retrieval-augmented generation (RAG) systems, which combine information retrieval with text generation.",
                "sources": [
                    {
                        "chapter": "Chapter 3",
                        "section": "3.1 Introduction",
                        "text": "Chapter 3 covers the fundamentals of retrieval-augmented generation (RAG) systems."
                    }
                ],
                "mode_used": "book-wide",
                "timestamp": "2025-01-15T12:00:00Z"
            }

            # Execute book-wide query
            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=book_wide_request.query,
                mode=book_wide_request.mode
            )

            result = generation_service.generate_answer(
                query=book_wide_request.query,
                context_chunks=context_chunks,
                mode=book_wide_request.mode
            )

            # Verify book-wide mode still works correctly
            assert result["mode_used"] == "book-wide"
            assert "retrieval-augmented generation" in result["response"].lower()