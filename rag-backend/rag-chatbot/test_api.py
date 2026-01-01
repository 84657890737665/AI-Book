import asyncio
import sys
import os

# Add the current directory to the Python path
sys.path.append(os.path.dirname(__file__))

def test_api_setup():
    """
    Test to verify that the API is properly set up with required environment variables.
    """
    print("Testing RAG Chatbot API setup...")
    
    # Check for required environment variables
    required_vars = [
        "OPENROUTER_API_KEY",
        "COHERE_API_KEY", 
        "QDRANT_URL",
        "QDRANT_API_KEY"
    ]
    
    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        print(f"[ERROR] Missing required environment variables: {', '.join(missing_vars)}")
        print("\nTo run the RAG Chatbot API, you need to set up these environment variables:")
        print("- OPENROUTER_API_KEY: for LLM access")
        print("- COHERE_API_KEY: for embedding generation")
        print("- QDRANT_URL: for vector database access")
        print("- QDRANT_API_KEY: for vector database authentication")
        print("- COLLECTION_NAME: name of the Qdrant collection (defaults to 'rag_pipeline')")
        print("\nCreate a .env file with these variables or set them in your environment.")
        return False

    print("[SUCCESS] All required environment variables are set")
    return True

def test_api_connection():
    """
    Test basic API functionality without making actual queries.
    """
    try:
        # Import the agent functionality
        from agent import AIAgentService, AgentConfiguration
        from data_retrieval import DataRetrievalService
        
        print("\n[SUCCESS] Successfully imported agent and data retrieval modules")

        # Try to initialize the retrieval service (this will fail without proper setup)
        try:
            retrieval_service = DataRetrievalService()
            print("[SUCCESS] Successfully connected to Qdrant vector database")
        except Exception as e:
            print(f"[WARNING] Could not connect to Qdrant vector database: {str(e)}")
            print("   This is expected if the database hasn't been set up or populated yet.")
            return False

        return True

    except ImportError as e:
        print(f"[ERROR] Failed to import required modules: {str(e)}")
        return False
    except Exception as e:
        print(f"[ERROR] Error during API connection test: {str(e)}")
        return False

async def test_sample_query():
    """
    Test a sample query if all prerequisites are met.
    """
    print("\nTesting sample query...")
    
    # Import the agent functionality
    from agent import run_agent_query

    # Test query
    query_text = "What is ROS2?"
    
    try:
        # Run the agent query (this is an async function, so we need to await it)
        response = await run_agent_query(
            query_text=query_text,
            context_scope="full_book"
        )
        
        print(f"\n[SUCCESS] Query successful!")
        print(f"Query: {query_text}")
        print(f"Response: {response.response_text}")
        print(f"Grounded in context: {response.grounded_in_context}")
        print(f"Confidence score: {response.confidence_score}")
        print(f"Citation references: {response.citation_references}")

        return True

    except Exception as e:
        print(f"[WARNING] Query failed (this is expected if the vector database is not set up): {str(e)}")
        return False

async def main():
    print("RAG Chatbot API Integration Test")
    print("="*50)
    
    # Test 1: Check setup
    setup_ok = test_api_setup()
    if not setup_ok:
        print("\n[ERROR] Setup test failed. Please configure the required environment variables.")
        return
    
    # Test 2: Test API connection
    connection_ok = test_api_connection()
    if not connection_ok:
        print("\n⚠️  Connection test had issues, but this may be expected if the database hasn't been set up yet.")
    
    # Test 3: Test sample query (only if connection is OK)
    if connection_ok:
        query_ok = await test_sample_query()
        if query_ok:
            print("\n[SUCCESS] All tests passed! The RAG Chatbot API is properly configured and working.")
        else:
            print("\n[WARNING] Query test failed, but this may be expected if the knowledge base hasn't been populated yet.")
    else:
        print("\n[WARNING] Skipping query test due to connection issues.")

    print("\n[NOTES]:")
    print("- If you see Qdrant connection errors, you need to set up and populate the vector database")
    print("- To populate the database, you would typically run the ingestion process first")
    print("- The ingestion process would involve running 'python ingest.py' or similar with your book content")

if __name__ == "__main__":
    asyncio.run(main())