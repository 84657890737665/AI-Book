import cohere
import logging
from typing import List, Dict, Any, Optional
from config.settings import settings
from src.services.retrieval_service import RetrievalService


class GenerationService:
    """
    Service to handle generation of responses using Cohere's language model.
    """
    
    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(settings.cohere_api_key)
        
        # Initialize retrieval service to get context
        self.retrieval_service = RetrievalService()
        
        # Setup logger
        self.logger = logging.getLogger(__name__)

    def generate_response(self, book_id: str, query_text: str, selected_text: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Generate a response for the given query using RAG.
        
        Args:
            book_id: The ID of the book to query
            query_text: The user's question
            selected_text: Optional text to focus the response on
        
        Returns:
            Dictionary containing the response and citations
        """
        try:
            # Retrieve relevant context for the query
            relevant_contexts = self.retrieval_service.retrieve_context(
                book_id=book_id, 
                query_text=query_text, 
                selected_text=selected_text,
                top_k=5
            )
            
            if not relevant_contexts:
                self.logger.warning(f"No relevant contexts found for query: '{query_text[:50]}...'")
                return {
                    "response": "I couldn't find any relevant information in the book to answer your question.",
                    "citations": [],
                    "context_used": []
                }
            
            # Format the context as a single string
            context_text = "\n".join([ctx.get("chunk_text", "") or ctx.get("text", "") for ctx in relevant_contexts])
            
            # Construct the prompt for the language model
            prompt = self._construct_prompt(query_text, context_text)
            
            # Generate the response using Cohere
            response = self.cohere_client.chat(
                message=prompt,
                model=settings.cohere_model_generation,
                temperature=0.3,  # Low temperature for factual accuracy
                max_tokens=500  # Limit response length
            )
            
            # Extract the generated text
            generated_text = response.text
            
            # Generate citations based on the retrieved contexts
            citations = self._generate_citations(relevant_contexts, query_text)
            
            self.logger.info(f"Generated response for query: '{query_text[:50]}...'")
            
            return {
                "response": generated_text,
                "citations": citations,
                "context_used": [ctx.get("chunk_text", "") for ctx in relevant_contexts]
            }
            
        except Exception as e:
            self.logger.error(f"Error generating response for query '{query_text[:50]}...': {str(e)}")
            return None

    def _construct_prompt(self, query: str, context: str) -> str:
        """
        Construct a prompt for the language model using the query and context.
        """
        prompt = (
            f"Based on the following context, please answer the question. "
            f"If the context doesn't contain enough information to answer the question, say so.\n\n"
            f"Context:\n{context}\n\n"
            f"Question: {query}\n\n"
            f"Answer:"
        )
        return prompt

    def _generate_citations(self, contexts: List[Dict[str, Any]], query: str) -> List[Dict[str, Any]]:
        """
        Generate citations from the retrieved contexts.
        """
        citations = []
        for context in contexts:
            citation = {
                "source_id": context.get("chunk_id", ""),
                "text": context.get("chunk_text", "") or context.get("text", ""),
                "score": context.get("score", 0.0),
                "metadata": context.get("metadata", {})
            }
            # Add the citation if it contains relevant information for the query
            if citation["text"].strip():  # Only add if there's actual text
                citations.append(citation)
        
        return citations

    def generate_summary(self, book_id: str, chapter_range: Optional[Dict[int, int]] = None) -> Optional[str]:
        """
        Generate a summary of the book or a specific chapter range.
        """
        try:
            # This is a simplified implementation - in practice, you'd want to retrieve 
            # more specific content for the chapters being summarized
            query_text = ("Provide a concise summary of the book. " +
                         f"Chapter range: {chapter_range}" if chapter_range else "the entire book")
            
            # For summary, we might want to retrieve more content
            relevant_contexts = self.retrieval_service.retrieve_context(
                book_id=book_id, 
                query_text=query_text, 
                top_k=10
            )
            
            if not relevant_contexts:
                return "Could not generate a summary as no content was found."
            
            # Format the context for summary
            context_text = "\n".join([ctx.get("chunk_text", "") for ctx in relevant_contexts[:5]])  # Limit to first 5 chunks
            
            prompt = (
                f"Please provide a concise summary of the following content:\n\n"
                f"{context_text}\n\n"
                f"Summary:"
            )
            
            response = self.cohere_client.chat(
                message=prompt,
                model=settings.cohere_model_generation,
                temperature=0.4,  # Slightly higher for creative summarization
                max_tokens=300
            )
            
            return response.text
            
        except Exception as e:
            self.logger.error(f"Error generating summary: {str(e)}")
            return None