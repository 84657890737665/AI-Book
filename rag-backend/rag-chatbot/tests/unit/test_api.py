"""Test for the FastAPI application."""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


def test_root_endpoint():
    """Test the root endpoint."""
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "RAG Pipeline API is running"
    assert data["version"] == "0.1.0"


def test_health_endpoint():
    """Test the health endpoint."""
    client = TestClient(app)
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"


def test_process_url_endpoint():
    """Test the process-url endpoint."""
    client = TestClient(app)
    # Test with a sample request
    request_data = {
        "url": "https://example.com"
    }
    response = client.post("/process-url", json=request_data)
    
    # Should return 200 with job information
    assert response.status_code == 200
    data = response.json()
    assert "job_id" in data
    assert data["status"] == "queued"
    assert "Processing started" in data["message"]