import os
from dotenv import load_dotenv
load_dotenv()

def test_agent_initialization():
    """Test if the agent can be initialized with the current configuration"""
    print("Testing agent initialization...")
    
    try:
        # Import the agent functionality
        from agent import AgentConfiguration, AIAgentService
        
        # Create agent configuration
        config = AgentConfiguration()
        
        print("Agent configuration created successfully")
        
        # Try to initialize the agent service
        print("Initializing agent service...")
        agent_service = AIAgentService(config)
        
        print("Agent service initialized successfully!")
        
        # Test a simple query
        print("Testing a simple query...")
        response = agent_service.run_query(
            query_text="What is the purpose of this system?",
            context_scope="full_book"
        )
        
        print(f"Query response: {response.response_text}")
        print(f"Grounded in context: {response.grounded_in_context}")
        print(f"Confidence score: {response.confidence_score}")
        
        return True
        
    except Exception as e:
        print(f"Error during agent initialization or query: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_agent_initialization()
    if success:
        print("\n[SUCCESS] Agent is working properly!")
    else:
        print("\n[ERROR] Agent initialization or query failed.")