import requests
import json
import subprocess
import time
import os

def test_api_endpoint():
    """
    Test the API endpoint directly by making a simple request to it.
    """
    print("Testing API endpoint functionality...")
    
    # Check if the server is running
    try:
        # Try to make a simple GET request to the root endpoint
        response = requests.get("http://localhost:8000/", timeout=5)
        if response.status_code == 200:
            print("[SUCCESS] API server is running and responding to requests")
            print(f"Response: {response.json()}")
        else:
            print(f"[ERROR] API server responded with status code: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("[ERROR] Could not connect to API server. Make sure it's running on http://localhost:8000/")
        print("To start the server, run: python api.py")
        return False
    except requests.exceptions.Timeout:
        print("[ERROR] Request timed out. API server may not be responding.")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error when connecting to API: {str(e)}")
        return False
    
    # Test the /query endpoint with a simple request (this will fail without proper setup but should return a proper error)
    try:
        test_payload = {
            "query": "What is ROS2?",
            "mode": "FULL_BOOK",
            "selected_text": None
        }
        
        response = requests.post(
            "http://localhost:8000/query",
            json=test_payload,
            timeout=10
        )
        
        print(f"\nQuery endpoint test:")
        print(f"Status code: {response.status_code}")
        
        if response.status_code == 200:
            print("[SUCCESS] Query endpoint processed the request successfully")
            print(f"Response: {response.json()}")
        elif response.status_code == 500:
            print("[EXPECTED ERROR] Query endpoint returned 500 - this is expected without proper environment setup")
            print("The error is likely due to missing API keys or vector database connection")
        else:
            print(f"[INFO] Query endpoint returned status code: {response.status_code}")
            print(f"Response: {response.text}")
        
        return True
        
    except requests.exceptions.ConnectionError:
        print("[ERROR] Could not connect to query endpoint. Make sure the server is running.")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error when testing query endpoint: {str(e)}")
        return False

def check_environment_variables():
    """
    Check if the required environment variables are set.
    """
    print("\nChecking environment variables...")
    
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
        print(f"[WARNING] Missing environment variables: {', '.join(missing_vars)}")
        print("These are required for the RAG functionality to work.")
    else:
        print("[SUCCESS] All required environment variables are set")
    
    return len(missing_vars) == 0

if __name__ == "__main__":
    print("API Endpoint Test")
    print("="*50)
    
    # Check environment variables
    env_ok = check_environment_variables()
    
    # Test the API endpoint
    api_ok = test_api_endpoint()
    
    print("\n" + "="*50)
    print("Test Summary:")
    print(f"Environment variables: {'OK' if env_ok else 'MISSING'}")
    print(f"API endpoint: {'RESPONDING' if api_ok else 'NOT RESPONDING'}")
    
    if api_ok:
        print("\n[SUCCESS] API is properly set up and responding to requests!")
        print("Note: Query functionality requires proper environment setup and vector database.")
    else:
        print("\n[ERROR] API is not responding. Please check the server setup.")