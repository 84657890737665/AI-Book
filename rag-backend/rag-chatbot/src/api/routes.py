from fastapi import APIRouter, HTTPException
from src.api.models import ChatRequest, ChatResponse, SourceDocument

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    # TODO: Connect this to the actual RAG pipeline
    # For now, we return a mock response to verify the API connectivity
    
    mock_sources = [
        SourceDocument(
            url="https://example.com/book/chapter1",
            title="Introduction to Physical AI",
            snippet="Physical AI combines traditional AI with physical interaction...",
            score=0.95
        )
    ]
    
    return ChatResponse(
        response=f"This is a mock response to your question: '{request.message}'. The RAG pipeline is not yet fully connected in this API route.",
        sources=mock_sources
    )

@router.get("/health")
async def health_check():
    return {"status": "ok", "service": "rag-chatbot"}
