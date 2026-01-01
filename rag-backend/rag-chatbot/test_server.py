import subprocess
import time
import requests
import json

def start_server():
    """Start the API server in the background"""
    print("Starting the API server...")
    # Start the server in the background
    process = subprocess.Popen(['python', 'api.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Wait a few seconds for the server to start
    time.sleep(5)
    
    return process

def test_server():
    """Test if the server is responding properly"""
    try:
        # Test the root endpoint
        response = requests.get("http://127.0.0.1:8000/", timeout=60)
        print(f"Root endpoint: Status {response.status_code}")
        print(f"Response: {response.json()}")
        
        # Test the query endpoint with a proper payload
        payload = {
            "query": "What is ROS2?",
            "mode": "FULL_BOOK",
            "selected_text": None
        }
        
        response = requests.post(
            "http://127.0.0.1:8000/query",
            json=payload,
            timeout=60
        )
        
        print(f"\nQuery endpoint: Status {response.status_code}")
        print(f"Response: {response.text}")
        
        return True
    except Exception as e:
        print(f"Error testing server: {str(e)}")
        return False

def main():
    print("Testing API server with proper payload...")
    
    # Start the server
    server_process = start_server()
    
    try:
        # Test the server
        success = test_server()
        
        if success:
            print("\n[SUCCESS] Server is responding correctly!")
        else:
            print("\n[ERROR] Server test failed.")
    finally:
        # Terminate the server process
        server_process.terminate()
        server_process.wait()
        print("Server stopped.")

if __name__ == "__main__":
    main()