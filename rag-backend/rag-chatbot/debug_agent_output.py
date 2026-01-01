import asyncio
import os
from dotenv import load_dotenv
from agent import AIAgentService, AgentConfiguration

async def debug_agent():
    print("Debugging Agent Output...")
    load_dotenv()
    
    config = AgentConfiguration()
    service = AIAgentService(config)
    
    # Simple query that shouldn't require RAG if instructions allow, 
    # but the current instructions are very strict.
    query = "Hello, who are you?"
    
    print(f"Running query: {query}")
    try:
        from agents import Runner
        # Test Runner.run directly to see output
        result = await Runner.run(service.agent, query)
        print(f"\nResult type: {type(result)}")
        print(f"Result attributes: {dir(result)}")
        
        if hasattr(result, 'final_output'):
            print(f"final_output: '{result.final_output}'")
        
        if hasattr(result, 'messages'):
            print(f"Last message: {result.messages[-1]}")
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(debug_agent())
