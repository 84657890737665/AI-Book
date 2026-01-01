from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, Union
import os
import sys
import logging
from datetime import datetime
from dotenv import load_dotenv
import re

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables explicitly from the current directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=env_path)

# sys.path modification removed as it was redundant

# Import the agent functionality
try:
    from agent import run_agent_query
except ImportError as e:
    logger.error(f"Error importing agent: {e}")
    print("Make sure the rag-backend/rag-chatbot directory exists and contains the agent.py file")
    raise

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot that connects to existing agent logic",
    version="1.0.0"
)

# Add CORS middleware for security
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Define request and response models
class QueryRequest(BaseModel):
    query: str
    mode: str  # "FULL_BOOK" or "SELECTED_TEXT"
    selected_text: Optional[str] = None


class QueryResponse(BaseModel):
    response: str
    status: str
    grounded_in_context: bool = None
    confidence_score: float = None
    citation_references: list = []

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    start_time = datetime.now()
    logger.info(f"Received query: {request.query[:50]}... with mode: {request.mode}")

    try:
        # Validate inputs manually since we removed field validators
        if not request.query or not request.query.strip():
            logger.warning("Empty query received")
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(request.query) > 1000:  # Example limit
            logger.warning(f"Query too long: {len(request.query)} characters")
            raise HTTPException(status_code=400, detail="Query exceeds maximum length of 1000 characters")

        if request.selected_text and len(request.selected_text) > 2000:  # Example limit
            logger.warning(f"Selected text too long: {len(request.selected_text)} characters")
            raise HTTPException(status_code=400, detail="Selected text exceeds maximum length of 2000 characters")

        # Basic sanitization to prevent injection
        if '<script' in request.query.lower() or 'javascript:' in request.query.lower():
            logger.warning("Potential script injection detected in query")
            raise HTTPException(status_code=400, detail="Query contains invalid content")

        if request.selected_text and ('<script' in request.selected_text.lower() or 'javascript:' in request.selected_text.lower()):
            logger.warning("Potential script injection detected in selected text")
            raise HTTPException(status_code=400, detail="Selected text contains invalid content")

        # Validate mode
        if request.mode not in ["FULL_BOOK", "SELECTED_TEXT"]:
            logger.warning(f"Invalid mode received: {request.mode}")
            raise HTTPException(status_code=400, detail="Mode must be 'FULL_BOOK' or 'SELECTED_TEXT'")

        # Map frontend mode to agent context_scope
        context_scope = "full_book" if request.mode == "FULL_BOOK" else "selected_text"

        # Prepare parameters for the agent
        agent_params = {
            "query_text": request.query,
            "context_scope": context_scope
        }

        # Add selected_text if provided and context_scope is selected_text
        if request.selected_text and context_scope == "selected_text":
            agent_params["selected_text_ref"] = request.selected_text
            logger.info(f"Processing query with selected text context: {request.selected_text[:50]}...")

        # Call the existing RAG agent
        agent_response = await run_agent_query(**agent_params)

        # Calculate response time
        response_time = (datetime.now() - start_time).total_seconds()
        logger.info(f"Query processed successfully in {response_time:.2f}s")

        # Return the response in the expected format
        return QueryResponse(
            response=agent_response.response_text,
            status="success",
            grounded_in_context=agent_response.grounded_in_context,
            confidence_score=agent_response.confidence_score,
            citation_references=agent_response.citation_references
        )
    except Exception as e:
        # Calculate response time for error case
        response_time = (datetime.now() - start_time).total_seconds()
        error_msg = str(e)
        logger.error(f"Error processing query after {response_time:.2f}s: {error_msg}", exc_info=True)
        
        # We raise an HTTPException to return a non-200 status
        raise HTTPException(status_code=500, detail=error_msg)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)