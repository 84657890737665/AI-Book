"""
RAG Pipeline - Data Retrieval and End-to-End Validation

This module connects to an existing Qdrant collection containing embedded book data,
performs similarity searches on user queries, and validates the results.
"""
import os
import argparse
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import numpy as np
from dotenv import load_dotenv


# Load environment variables explicitly from the current directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=env_path)


@dataclass
class QueryEmbedding:
    """Represents the embedding vector created from a text query for similarity search in Qdrant."""
    id: str
    text: str
    vector: List[float]
    created_at: datetime
    model_used: str


@dataclass
class SearchResult:
    """Represents a retrieved chunk with its similarity score, text content, metadata, and source reference."""
    id: str
    chunk_id: str
    score: float
    text: str
    source_url: str
    section_title: str
    metadata: Dict[str, Any]
    created_at: datetime


@dataclass
class ValidationReport:
    """Represents the outcome of a validation run, including accuracy metrics and any issues found."""
    id: str
    query: str
    results_validated: int
    accuracy_score: float
    issues_found: List[str]
    created_at: datetime
    validated_by: str


@dataclass
class QdrantConnection:
    """Represents the connection to the Qdrant Cloud collection containing the stored embeddings."""
    id: str
    url: str
    api_key: str
    collection_name: str
    status: str
    last_connected: datetime


class DataRetrievalService:
    """Main service class for data retrieval and validation."""
    
    def __init__(self):
        """Initialize the service with required clients and configurations."""
        # Initialize Cohere client
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(self.cohere_api_key)
        
        # Initialize Qdrant client
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "rag_pipeline")
        
        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")
        
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            prefer_grpc=False,
            timeout=60  # Increased timeout to handle network latency
        )
        
        # Validate collection exists
        try:
            self.qdrant_client.get_collection(self.collection_name)
            logging.info(f"Connected to Qdrant collection: {self.collection_name}")
        except Exception as e:
            raise ValueError(f"Collection {self.collection_name} does not exist in Qdrant: {str(e)}")
    
    def generate_query_embedding(self, query_text: str, model_name: str = None) -> QueryEmbedding:
        """
        Generate an embedding for the given query text using Cohere.
        
        Args:
            query_text: The text to generate an embedding for
            model_name: The Cohere model to use (defaults to the one used during ingestion)
        
        Returns:
            QueryEmbedding object containing the embedding vector
        """
        if not model_name:
            # Default to the multilingual model which was likely used during ingestion
            model_name = "embed-multilingual-v2.0"
        
        start_time = datetime.now()
        
        try:
            # Generate embedding using Cohere with retry logic for rate limits
            max_retries = 5
            response = None
            
            for attempt in range(max_retries):
                try:
                    response = self.cohere_client.embed(
                        texts=[query_text],
                        model=model_name,
                        input_type="search_query"
                    )
                    break
                except Exception as e:
                    error_msg = str(e)
                    if "429" in error_msg or "Too Many Requests" in error_msg or "rate limit" in error_msg.lower():
                        if attempt == max_retries - 1:
                            raise
                        import time
                        wait_time = 2 * (2 ** attempt)
                        logging.warning(f"Cohere rate limit hit. Retrying in {wait_time}s (Attempt {attempt + 1}/{max_retries})")
                        time.sleep(wait_time)
                    else:
                        raise

            if not response:
                raise ValueError("Failed to get response from Cohere after retries")

            # Extract the embedding vector
            embedding_vector = response.embeddings[0]
            
            # Create and return QueryEmbedding object
            query_embedding = QueryEmbedding(
                id=f"query_emb_{int(start_time.timestamp())}",
                text=query_text,
                vector=embedding_vector,
                created_at=start_time,
                model_used=model_name
            )
            
            logging.info(f"Generated embedding for query in {datetime.now() - start_time}")
            return query_embedding
            
        except Exception as e:
            logging.error(f"Error generating embedding for query: {str(e)}")
            raise
    
    def search_similar_chunks(self, query_embedding: QueryEmbedding, top_k: int = 5) -> List[SearchResult]:
        """
        Perform a similarity search in Qdrant to find similar chunks.
        
        Args:
            query_embedding: The embedding to search for similar vectors to
            top_k: Number of top results to return (default: 5)
        
        Returns:
            List of SearchResult objects
        """
        start_time = datetime.now()
        
        try:
            # Perform the search in Qdrant
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding.vector,
                limit=top_k
            ).points
            
            # Convert Qdrant results to our SearchResult objects
            results = []
            for result in search_results:
                payload = result.payload or {}
                
                search_result = SearchResult(
                    id=result.id,
                    chunk_id=payload.get("chunk_id", "unknown"),
                    score=result.score,
                    text=payload.get("chunk_text", ""),  # Use actual chunk content
                    source_url=payload.get("source_url", ""),
                    section_title=payload.get("section_title", ""),
                    metadata=payload.get("additional_metadata", {}),
                    created_at=datetime.now()
                )
                
                results.append(search_result)
            
            logging.info(f"Found {len(results)} similar chunks in {datetime.now() - start_time}")
            return results
            
        except Exception as e:
            logging.error(f"Error performing similarity search: {str(e)}")
            raise
    
    def validate_results(self, search_results: List[SearchResult], query: str) -> ValidationReport:
        """
        Validate the search results for correctness and completeness.
        
        Args:
            search_results: List of SearchResult objects to validate
            query: The original query that generated these results
        
        Returns:
            ValidationReport object with validation results
        """
        start_time = datetime.now()
        issues_found = []
        accuracy_score = 1.0  # Start with perfect score
        
        # Validate each result
        for i, result in enumerate(search_results):
            # Check if text content exists
            if not result.text or result.text.strip() == "":
                issues_found.append(f"Result {i}: Missing or empty text content")
                accuracy_score -= 0.1  # Reduce accuracy for each issue
            
            # Check if source URL is valid
            if not result.source_url or not result.source_url.startswith(("http://", "https://")):
                issues_found.append(f"Result {i}: Invalid or missing source URL: {result.source_url}")
                accuracy_score -= 0.1  # Reduce accuracy for each issue
            
            # Check if score is reasonable (between 0 and 1)
            if not (0 <= result.score <= 1):
                issues_found.append(f"Result {i}: Score out of range [0,1]: {result.score}")
                accuracy_score -= 0.2  # Reduce accuracy more for score issues
            
            # Check if section title exists
            if not result.section_title or result.section_title.strip() == "":
                issues_found.append(f"Result {i}: Missing or empty section title")
                accuracy_score -= 0.05  # Reduce accuracy slightly for missing title
        
        # Ensure accuracy score doesn't go below 0
        accuracy_score = max(0.0, accuracy_score)
        
        # Create and return validation report
        validation_report = ValidationReport(
            id=f"validation_{int(start_time.timestamp())}",
            query=query,
            results_validated=len(search_results),
            accuracy_score=accuracy_score,
            issues_found=issues_found,
            created_at=datetime.now(),
            validated_by="DataRetrievalService"
        )
        
        logging.info(f"Validation completed in {datetime.now() - start_time}, "
                     f"accuracy: {accuracy_score:.2f}, issues: {len(issues_found)}")
        
        return validation_report
    
    def run_test_flow(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Execute the complete end-to-end test flow.
        
        Args:
            query: The search query to test
            top_k: Number of results to return (default: 5)
        
        Returns:
            Dictionary with test flow results
        """
        start_time = datetime.now()
        
        result = {
            "test_flow_result": {
                "query": query,
                "top_k": top_k,
                "embedding_generation": {
                    "success": False,
                    "processing_time_ms": 0,
                    "error": None
                },
                "similarity_search": {
                    "success": False,
                    "results_count": 0,
                    "search_time_ms": 0,
                    "error": None
                },
                "result_validation": {
                    "success": False,
                    "accuracy_score": 0.0,
                    "validation_time_ms": 0,
                    "error": None
                },
                "overall_status": "failure",
                "completed_at": None
            }
        }
        
        try:
            # Step 1: Generate query embedding
            embedding_start = datetime.now()
            query_embedding = self.generate_query_embedding(query)
            result["test_flow_result"]["embedding_generation"]["success"] = True
            result["test_flow_result"]["embedding_generation"]["processing_time_ms"] = (
                datetime.now() - embedding_start
            ).total_seconds() * 1000
            
            # Step 2: Perform similarity search
            search_start = datetime.now()
            search_results = self.search_similar_chunks(query_embedding, top_k)
            result["test_flow_result"]["similarity_search"]["success"] = True
            result["test_flow_result"]["similarity_search"]["results_count"] = len(search_results)
            result["test_flow_result"]["similarity_search"]["search_time_ms"] = (
                datetime.now() - search_start
            ).total_seconds() * 1000
            
            # Step 3: Validate results
            validation_start = datetime.now()
            validation_report = self.validate_results(search_results, query)
            result["test_flow_result"]["result_validation"]["success"] = True
            result["test_flow_result"]["result_validation"]["accuracy_score"] = validation_report.accuracy_score
            result["test_flow_result"]["result_validation"]["validation_time_ms"] = (
                datetime.now() - validation_start
            ).total_seconds() * 1000
            
            # Determine overall status
            if (result["test_flow_result"]["embedding_generation"]["success"] and
                result["test_flow_result"]["similarity_search"]["success"] and
                result["test_flow_result"]["result_validation"]["success"]):
                
                if validation_report.accuracy_score >= 0.8:
                    result["test_flow_result"]["overall_status"] = "success"
                elif validation_report.accuracy_score >= 0.5:
                    result["test_flow_result"]["overall_status"] = "partial_success"
                else:
                    result["test_flow_result"]["overall_status"] = "failure"
            else:
                result["test_flow_result"]["overall_status"] = "failure"
                
            result["test_flow_result"]["completed_at"] = datetime.now().isoformat()
            
            # Log summary
            logging.info(f"Test flow completed for query '{query[:50]}...' - "
                         f"Status: {result['test_flow_result']['overall_status']}, "
                         f"Accuracy: {validation_report.accuracy_score:.2f}, "
                         f"Time: {(datetime.now() - start_time).total_seconds():.2f}s")
            
            return result
            
        except Exception as e:
            error_msg = str(e)
            logging.error(f"Error in test flow: {error_msg}")
            
            # Update error information in result
            if not result["test_flow_result"]["embedding_generation"]["success"]:
                result["test_flow_result"]["embedding_generation"]["error"] = error_msg
            elif not result["test_flow_result"]["similarity_search"]["success"]:
                result["test_flow_result"]["similarity_search"]["error"] = error_msg
            elif not result["test_flow_result"]["result_validation"]["success"]:
                result["test_flow_result"]["result_validation"]["error"] = error_msg
            
            result["test_flow_result"]["overall_status"] = "failure"
            result["test_flow_result"]["completed_at"] = datetime.now().isoformat()
            
            return result


def main():
    """Main entry point for the data retrieval service."""
    # Set up logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    parser = argparse.ArgumentParser(description="RAG Pipeline - Data Retrieval and End-to-End Validation")
    parser.add_argument("--query", type=str, help="The search query to validate")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results to return (default: 5)")
    parser.add_argument("--test-flow", action="store_true", help="Run the complete end-to-end validation flow")
    
    args = parser.parse_args()
    
    # Create the service instance
    service = DataRetrievalService()
    
    if args.test_flow:
        # Run a default test query if none provided
        test_query = args.query or "What are the key principles of Physical AI and Humanoid Robotics?"
        print(f"Running end-to-end test flow for query: '{test_query}'")
        
        result = service.run_test_flow(test_query, args.top_k)
        print(f"\nTest Flow Result: {result}")
        
        # Print validation report summary
        flow_result = result["test_flow_result"]
        print(f"\nSummary:")
        print(f"- Status: {flow_result['overall_status']}")
        print(f"- Query: {flow_result['query']}")
        print(f"- Results retrieved: {flow_result['similarity_search']['results_count']}")
        print(f"- Accuracy score: {flow_result['result_validation']['accuracy_score']:.2f}")
        
        if flow_result['result_validation']['success']:
            print(f"- Issues found: {len(flow_result['result_validation'].get('issues_found', []))}")
    
    elif args.query:
        # Run a single query
        print(f"Processing query: '{args.query}'")
        
        result = service.run_test_flow(args.query, args.top_k)
        flow_result = result["test_flow_result"]
        
        print(f"\nResults:")
        print(f"- Status: {flow_result['overall_status']}")
        print(f"- Results retrieved: {flow_result['similarity_search']['results_count']}")
        print(f"- Accuracy score: {flow_result['result_validation']['accuracy_score']:.2f}")
        
        # Print the actual results
        service_instance = DataRetrievalService()
        query_embedding = service_instance.generate_query_embedding(args.query)
        search_results = service_instance.search_similar_chunks(query_embedding, args.top_k)
        
        print(f"\nTop {args.top_k} Results:")
        for i, res in enumerate(search_results, 1):
            print(f"{i}. Score: {res.score:.3f}")
            print(f"   Source: {res.source_url}")
            print(f"   Title: {res.section_title}")
            print(f"   Content: {res.text[:200]}{'...' if len(res.text) > 200 else ''}")
            print()
    
    else:
        print("Please provide either --query or --test-flow option.")
        parser.print_help()


if __name__ == "__main__":
    main()