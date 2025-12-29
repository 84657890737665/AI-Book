from pydantic import BaseModel
from typing import List, Optional, Dict, Any

class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    history: Optional[List[ChatMessage]] = []
    
class SourceDocument(BaseModel):
    url: str
    title: Optional[str] = None
    snippet: str
    score: Optional[float] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[SourceDocument] = []
