from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from src.api import auth_routes
from src.api import book_routes
from src.api import query_routes
from src.api import embed_routes

# Create the FastAPI app
app = FastAPI(
    title="Physical AI and Humanoid Robotics - RAG Backend",
    description="Backend API for the interactive digital book on Physical AI and Humanoid Robotics",
    version="1.0.0",
)

# Initialize the rate limiter
limiter = Limiter(key_func=get_remote_address)

# Set up rate limiting globally
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware - configured for local frontend development
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",      # Default React/Vite dev server
        "http://127.0.0.1:3000",
        "http://localhost:5173",      # Alternative Vite port
        "http://localhost:8000",      # If serving frontend via FastAPI static
        # Add your production frontend URL later, e.g.:
        # "https://physical-ai-book.yourdomain.com"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],  # In production, restrict to specific headers like Authorization, Content-Type
)

# Include API routers with proper prefixes and tags
app.include_router(auth_routes.router, prefix="/auth", tags=["Authentication"])
app.include_router(book_routes.router, prefix="/books", tags=["Books"])
app.include_router(query_routes.router, prefix="/query", tags=["Query"])
app.include_router(embed_routes.router, prefix="/embed", tags=["Embed"])

# API documentation redirects (optional but nice UX)
@app.get("/", tags=["Docs"])
async def root():
    return {
        "message": "Welcome to Physical AI and Humanoid Robotics RAG Backend",
        "docs_url": "/docs",
        "redoc_url": "/redoc"
    }

# Health check endpoint
@app.get("/health", tags=["Health"])
async def health_check():
    return {"status": "healthy", "service": "rag-backend"}