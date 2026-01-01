"""
RAG Pipeline - AI Agent with Retrieval Capabilities

This module integrates the OpenAI Agents SDK with the existing RAG pipeline 
to answer questions grounded in the book's embedded content.
"""
import os
import argparse
import logging
import asyncio
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables explicitly from the current directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=env_path)

from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel
from openai import AsyncOpenAI


# Configuration API
client = AsyncOpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1"
)

# OPENROUTER Model
third_party_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free"
)

# Import the retrieval functionality from data-retrieval.py
try:
    from data_retrieval import DataRetrievalService
except ImportError:
    # If the module is in a different location, try alternative import
    try:
        from rag_chatbot.data_retrieval import DataRetrievalService
    except ImportError:
        logging.warning("DataRetrievalService not found. Make sure data-retrieval.py is available.")


@function_tool
def retrieve_context_tool(query: str, top_k: int = 5) -> str:
    """
    Retrieve relevant context from the knowledge base based on the query.

    Args:
        query: The query to search for in the knowledge base
        top_k: Number of top results to return (default: 5)

    Returns:
        A string containing the relevant context chunks
    """
    # Initialize the retrieval service
    retrieval_service = DataRetrievalService()

    # Generate embedding for the query
    query_embedding = retrieval_service.generate_query_embedding(query)

    # Perform similarity search
    search_results = retrieval_service.search_similar_chunks(
        query_embedding,
        top_k=top_k
    )

    # Format the context for the agent
    context_chunks = []
    for result in search_results:
        chunk = f"Source: {result.source_url} | Section: {result.section_title}\nContent: {result.text[:500]}..."
        context_chunks.append(chunk)

    context_str = "\n\n".join(context_chunks)

    return context_str



# Environment variables loaded at top of file


@dataclass
class AgentConfiguration:
    """Represents the settings and parameters that control the agent's behavior."""
    assistant_id: Optional[str] = None
    model: str = "mistralai/devstral-2512:free"
    grounding_required: bool = True
    confidence_threshold: float = 0.7
    max_tokens: int = 1000
    temperature: float = 0.3
    retrieval_top_k: int = 5
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()


@dataclass
class AgentQuery:
    """Represents an incoming query to the AI agent with context about the source and requirements."""
    query_text: str
    context_scope: str = "full_book"  # "full_book" or "selected_text"
    selected_text_ref: Optional[str] = None
    user_id: Optional[str] = None
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()
        
        if self.context_scope not in ["full_book", "selected_text"]:
            raise ValueError("context_scope must be 'full_book' or 'selected_text'")
        
        if self.context_scope == "selected_text" and not self.selected_text_ref:
            raise ValueError("selected_text_ref is required when context_scope is 'selected_text'")


@dataclass
class RetrievedContext:
    """Represents the context retrieved from Qdrant based on the agent query."""
    query_id: str
    context_chunks: List[Dict[str, Any]]
    retrieval_score: float
    source_documents: List[str]
    retrieved_at: datetime = None

    def __post_init__(self):
        if self.retrieved_at is None:
            self.retrieved_at = datetime.now()


@dataclass
class AgentResponse:
    """Represents an agent response that is strictly based on retrieved context."""
    query_id: str
    response_text: str
    grounded_in_context: bool
    citation_references: List[str]
    confidence_score: float
    processing_steps: List[Dict[str, Any]]
    generated_at: datetime = None

    def __post_init__(self):
        if self.generated_at is None:
            self.generated_at = datetime.now()


@dataclass
class QueryContext:
    """Represents the intermediate representation of a user query."""
    query_id: str
    processed_query: str
    retrieval_params: Dict[str, Any]
    context_requirements: Dict[str, Any]
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()


@dataclass
class TraceLog:
    """Represents the audit trail of the agent's decision-making process."""
    query_id: str
    timestamp: datetime
    component: str
    step: str
    details: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None


class AIAgentService:
    """Main service class for the AI agent with retrieval capabilities."""
    
    def __init__(self, config: AgentConfiguration):
        """Initialize the AI agent service with required clients and configurations."""
        # Verify OpenRouter API key is available
        self.openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        if not self.openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")

        self.config = config

        # Initialize the retrieval service from Spec-2
        try:
            self.retrieval_service = DataRetrievalService()
        except Exception as e:
            logging.error(f"Failed to initialize retrieval service: {str(e)}")
            raise

        # Initialize the OpenAI Agent with the RAG tool
        self.agent = Agent(
            name="Book Content Assistant",
            instructions=self._get_agent_instructions(),
            model=third_party_model,
            tools=[retrieve_context_tool],
        )

    def _get_agent_instructions(self) -> str:
        """Get the instructions for the agent."""
        return """
        You are an AI assistant helping users understand content from a book on Physical AI and Humanoid Robotics.
        Your responses must be grounded strictly in the context provided by the retrieval system.
        Do not fabricate information or make claims not supported by the provided context.
        When answering, please cite the source of your information.
        """

    async def retrieve_context(self, query: AgentQuery) -> RetrievedContext:
        """
        Retrieve relevant context from Qdrant based on the agent query.
        
        Args:
            query: The agent query to retrieve context for
            
        Returns:
            RetrievedContext object containing the retrieved context
        """
        start_time = datetime.now()
        
        try:
            # Generate embedding for the query
            query_embedding = self.retrieval_service.generate_query_embedding(query.query_text)
            
            # Perform similarity search
            search_results = self.retrieval_service.search_similar_chunks(
                query_embedding, 
                top_k=self.config.retrieval_top_k
            )
            
            # Convert search results to context chunks
            context_chunks = []
            source_documents = set()
            total_score = 0.0
            
            for result in search_results:
                chunk = {
                    "id": result.id,
                    "chunk_id": result.chunk_id,
                    "score": result.score,
                    "text": result.text,
                    "source_url": result.source_url,
                    "section_title": result.section_title,
                    "metadata": result.metadata
                }
                
                context_chunks.append(chunk)
                source_documents.add(result.source_url)
                total_score += result.score
            
            # Calculate average retrieval score
            avg_score = total_score / len(search_results) if search_results else 0.0
            
            # Create and return RetrievedContext object
            retrieved_context = RetrievedContext(
                query_id=query.id if hasattr(query, 'id') else f"query_{int(start_time.timestamp())}",
                context_chunks=context_chunks,
                retrieval_score=avg_score,
                source_documents=list(source_documents),
                retrieved_at=datetime.now()
            )
            
            logging.info(f"Retrieved context with {len(context_chunks)} chunks in {datetime.now() - start_time}")
            return retrieved_context
            
        except Exception as e:
            logging.error(f"Error retrieving context: {str(e)}")
            raise
    
    def validate_response_grounding(self, response_text: str, retrieved_context: RetrievedContext) -> Dict[str, Any]:
        """
        Validate that the agent response is grounded in the retrieved context.
        
        Args:
            response_text: The response text to validate
            retrieved_context: The context that was retrieved for the query
            
        Returns:
            Dictionary with validation results
        """
        start_time = datetime.now()
        
        # Check if response contains information from the retrieved context
        context_texts = [chunk["text"] for chunk in retrieved_context.context_chunks]
        context_str = " ".join(context_texts)
        
        # Simple validation: check if response contains key phrases from context
        response_lower = response_text.lower()
        context_lower = context_str.lower()
        
        # Count how many context snippets appear in the response
        matching_snippets = 0
        total_snippets = len(context_texts)
        
        for snippet in context_texts:
            if snippet.lower() in response_lower:
                matching_snippets += 1
        
        grounding_score = matching_snippets / total_snippets if total_snippets > 0 else 0.0
        
        # Check for citation patterns in the response
        has_citations = any(pattern in response_text.lower() for pattern in [
            "according to", "as stated", "source:", "cited", "reference"
        ])
        
        validation_result = {
            "is_valid": grounding_score > 0.1 or has_citations,  # At least 10% overlap or has citations
            "grounding_score": grounding_score,
            "has_citations": has_citations,
            "matching_snippets": matching_snippets,
            "total_snippets": total_snippets,
            "validation_time_ms": (datetime.now() - start_time).total_seconds() * 1000
        }
        
        logging.info(f"Response validation completed with score: {grounding_score:.2f}")
        return validation_result
    
    async def generate_agent_response(self, query: AgentQuery) -> AgentResponse:
        """
        Generate a response using the OpenAI agent based on the query.

        Args:
            query: The original agent query

        Returns:
            AgentResponse object containing the generated response
        """
        start_time = datetime.now()
        processing_steps = []

        try:
            # Use the agent to generate the response
            # Pass the top_k from config to the tool call via kwargs if the SDK supports it
            # or rely on the agent to pass it if it learns it from instructions.
            # For now, we'll ensure the tool uses the config value.
            result = await Runner.run(self.agent, query.query_text)
            response_text = result.final_output

            # Since we're using the tool approach, we need to retrieve context for validation
            retrieved_context = await self.retrieve_context(query)

            # Validate that the response is grounded in the context
            validation_result = self.validate_response_grounding(response_text, retrieved_context)

            # Extract citation references from the response
            citation_refs = self.extract_citations(response_text, retrieved_context)

            # Create processing steps log
            processing_steps.append({
                "step": "context_retrieval",
                "description": f"Retrieved {len(retrieved_context.context_chunks)} context chunks",
                "duration_ms": 0  # Placeholder, actual time captured separately
            })

            processing_steps.append({
                "step": "response_generation",
                "description": "Generated response using OpenAI agent",
                "duration_ms": (datetime.now() - start_time).total_seconds() * 1000
            })

            # Create and return the AgentResponse object
            agent_response = AgentResponse(
                query_id=query.id if hasattr(query, 'id') else f"query_{int(start_time.timestamp())}",
                response_text=response_text,
                grounded_in_context=validation_result["is_valid"],
                citation_references=citation_refs,
                confidence_score=validation_result["grounding_score"],
                processing_steps=processing_steps
            )

            logging.info(f"Generated agent response in {datetime.now() - start_time}")
            return agent_response

        except Exception as e:
            logging.error(f"Error generating agent response: {str(e)}")
            raise
    
    def extract_citations(self, response_text: str, retrieved_context: RetrievedContext) -> List[str]:
        """
        Extract citation references from the response based on the retrieved context.
        
        Args:
            response_text: The response text to extract citations from
            retrieved_context: The context that was retrieved for the query
            
        Returns:
            List of citation references
        """
        citations = set()
        
        # Look for sources mentioned in the response
        for chunk in retrieved_context.context_chunks:
            source_url = chunk.get("source_url", "")
            section_title = chunk.get("section_title", "")
            
            # Check if the source is mentioned in the response
            if source_url and source_url in response_text:
                citations.add(source_url)
            if section_title and section_title.lower() in response_text.lower():
                citations.add(section_title)
        
        return list(citations)
    
    async def run_query(self, query_text: str, context_scope: str = "full_book", selected_text_ref: str = None) -> AgentResponse:
        """
        Execute a complete query through the agent pipeline.
        
        Args:
            query_text: The query text to process
            context_scope: The scope of the query ("full_book" or "selected_text")
            selected_text_ref: Reference to specific text if context_scope is "selected_text"
            
        Returns:
            AgentResponse object with the complete response
        """
        start_time = datetime.now()
        
        # Create the agent query
        query = AgentQuery(
            query_text=query_text,
            context_scope=context_scope,
            selected_text_ref=selected_text_ref
        )
        
        # Generate response using the agent (the agent will handle context retrieval via tools)
        agent_response = await self.generate_agent_response(query)

        logging.info(f"Complete query processed in {datetime.now() - start_time}")
        return agent_response


async def run_agent_query(query_text: str, context_scope: str = "full_book", selected_text_ref: str = None, agent_config: Dict[str, Any] = None) -> AgentResponse:
    """
    Main function to run a query through the AI agent.
    
    Args:
        query_text: The query text to process
        context_scope: The scope of the query ("full_book" or "selected_text")
        selected_text_ref: Reference to specific text if context_scope is "selected_text"
        agent_config: Optional configuration for the agent
        
    Returns:
        AgentResponse object with the complete response
    """
    # Set up logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # Create agent configuration
    config = AgentConfiguration(**(agent_config or {}))
    
    # Initialize the agent service
    agent_service = AIAgentService(config)
    
    # Run the query
    response = await agent_service.run_query(query_text, context_scope, selected_text_ref)
    
    return response


def main():
    """Main entry point for the AI agent service."""
    parser = argparse.ArgumentParser(description="RAG Pipeline - AI Agent with Retrieval Capabilities")
    parser.add_argument("--query", type=str, required=True, help="The question to ask the agent")
    parser.add_argument("--context-scope", type=str, default="full_book", 
                       choices=["full_book", "selected_text"], 
                       help="Scope of the query (default: full_book)")
    parser.add_argument("--text-ref", type=str, help="Reference to specific text when context-scope is selected_text")
    parser.add_argument("--grounding-required", type=bool, default=True, 
                       help="Whether responses must be grounded in retrieved context (default: True)")
    parser.add_argument("--confidence-threshold", type=float, default=0.7, 
                       help="Minimum confidence score for context retrieval (default: 0.7)")
    parser.add_argument("--top-k", type=int, default=5, 
                       help="Number of context chunks to retrieve (default: 5)")
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.context_scope == "selected_text" and not args.text_ref:
        parser.error("--text-ref is required when --context-scope is 'selected_text'")
    
    # Set up logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    try:
        # Create agent configuration
        config = {
            "grounding_required": args.grounding_required,
            "confidence_threshold": args.confidence_threshold,
            "retrieval_top_k": args.top_k
        }
        
        # Run the agent query
        response = asyncio.run(run_agent_query(
            query_text=args.query,
            context_scope=args.context_scope,
            selected_text_ref=args.text_ref,
            agent_config=config
        ))
        
        # Print the response
        print(f"\nQuery: {args.query}")
        print(f"Response: {response.response_text}")
        print(f"Grounded in context: {response.grounded_in_context}")
        print(f"Confidence score: {response.confidence_score:.2f}")
        
        if response.citation_references:
            print(f"Citations: {', '.join(response.citation_references)}")
        
        # Print processing steps
        print(f"\nProcessing steps:")
        for step in response.processing_steps:
            print(f"  - {step['step']}: {step['description']} ({step['duration_ms']:.2f}ms)")
    
    except Exception as e:
        logging.error(f"Error running agent query: {str(e)}")
        print(f"Error: {str(e)}")


if __name__ == "__main__":
    main()