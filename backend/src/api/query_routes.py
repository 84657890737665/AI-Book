from fastapi import APIRouter, HTTPException, status, Depends, BackgroundTasks
from typing import Optional
from datetime import datetime
import uuid

from src.models.user_query import UserQuery, QueryMode
from src.services.generation_service import GenerationService
from config.database import SessionLocal
from src.api.auth_routes import get_current_user

router = APIRouter()
generation_service = GenerationService()


@router.post("/{book_id}")
async def query_book(
    book_id: str,
    query: str,
    mode: QueryMode = QueryMode.FULL_BOOK,
    selected_text: Optional[str] = None,
    current_user: str = Depends(get_current_user),
    background_tasks: BackgroundTasks = None
):
    """
    Query the book content using RAG.
    
    Args:
        book_id: The ID of the book to query
        query: The question or query text
        mode: Query mode - full_book or selected_text
        selected_text: Text to focus on if mode is selected_text
    """
    # Validate input
    if not query or len(query.strip()) == 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Query cannot be empty"
        )
    
    if mode == QueryMode.SELECTED_TEXT and (not selected_text or len(selected_text.strip()) == 0):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Selected text is required when mode is selected_text"
        )
    
    # Generate response using RAG
    start_time = datetime.utcnow()
    response_data = generation_service.generate_response(
        book_id=book_id,
        query_text=query,
        selected_text=selected_text if mode == QueryMode.SELECTED_TEXT else None
    )
    
    if not response_data:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error generating response"
        )
    
    # Calculate response time
    response_time = (datetime.utcnow() - start_time).total_seconds()
    
    # Store the query and response in the database
    db = SessionLocal()
    try:
        user_query = UserQuery(
            user_id=current_user,
            book_id=book_id,
            query_text=query,
            query_mode=mode,
            selected_text=selected_text if mode == QueryMode.SELECTED_TEXT else None,
            retrieved_context=response_data["context_used"],
            response_text=response_data["response"],
            citation_references=response_data["citations"],
            query_latency=int(response_time * 1000)  # Convert to milliseconds
        )
        
        db.add(user_query)
        db.commit()
        db.refresh(user_query)
        
        return {
            "response": response_data["response"],
            "citations": response_data["citations"],
            "latency_ms": int(response_time * 1000),
            "query_id": user_query.id
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error storing query: {str(e)}"
        )
    finally:
        db.close()