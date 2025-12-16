from fastapi import APIRouter, HTTPException, status, Depends
from typing import Optional

from config.database import SessionLocal
from src.models.book_content import BookContent
from src.api.auth_routes import get_current_user
from src.pipeline.docusaurus_embedding_pipeline import DocusaurusEmbeddingPipeline

router = APIRouter()


@router.get("/{book_id}")
async def get_embeddable_chatbot(
    book_id: str,
    current_user: str = Depends(get_current_user)
):
    """
    Retrieve the HTML/JS code to embed the chatbot in a book.

    This returns the necessary code to embed the chatbot in digital book formats
    like PDF or EPUB to enable in-book querying.
    """
    db = SessionLocal()
    try:
        book = db.query(BookContent).filter(BookContent.id == book_id).first()

        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )

        # Check if book processing is completed
        if book.processing_status != "COMPLETED":
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Book processing is not completed. Cannot embed chatbot in an unprocessed book."
            )

        # Return the embeddable HTML/JS code
        embed_code = f"""
<div id="rag-chatbot-container">
  <div id="rag-chatbot-header">
    <h3>Ask about "{book.title}"</h3>
  </div>
  <div id="rag-chatbot-messages"></div>
  <div id="rag-chatbot-input-area">
    <input type="text" id="rag-query-input" placeholder="Ask a question about this book...">
    <button id="rag-send-btn">Send</button>
  </div>
</div>

<script>
// Simple chat interface implementation
const container = document.getElementById('rag-chatbot-container');
const messageArea = document.getElementById('rag-chatbot-messages');
const inputField = document.getElementById('rag-query-input');
const sendButton = document.getElementById('rag-send-btn');

// Function to add message to the chat
function addMessage(message, isUser = false) {{
  const messageElement = document.createElement('div');
  messageElement.classList.add(isUser ? 'user-message' : 'bot-message');
  messageElement.textContent = message;
  messageArea.appendChild(messageElement);
  // Scroll to bottom
  messageArea.scrollTop = messageArea.scrollHeight;
}}

// Send query to backend
async function sendQuery() {{
  const query = inputField.value.trim();
  if (!query) return;

  // Show user message
  addMessage(query, true);
  inputField.value = '';

  try {{
    // Call the backend API
    const response = await fetch('/query/{book_id}', {{
      method: 'POST',
      headers: {{
        'Content-Type': 'application/json',
        'Authorization': 'Bearer ' + localStorage.getItem('access_token')  // Assuming token is stored
      }},
      body: JSON.stringify({{
        query: query,
        mode: 'full_book'  // Default to full book mode
      }})
    }});

    if (!response.ok) {{
      throw new Error('Network response was not ok');
    }}

    const data = await response.json();
    addMessage(data.response);
  }} catch (error) {{
    console.error('Error sending query:', error);
    addMessage('Sorry, there was an error processing your request.');
  }}
}}

// Event listeners
sendButton.addEventListener('click', sendQuery);
inputField.addEventListener('keypress', (e) => {{
  if (e.key === 'Enter') sendQuery();
}});

// Add some basic CSS dynamically (in a real implementation, this would be loaded from a CSS file)
const style = document.createElement('style');
style.textContent = `
  #rag-chatbot-container {{
    border: 1px solid #ccc;
    border-radius: 8px;
    padding: 16px;
    font-family: Arial, sans-serif;
    max-width: 600px;
    margin: 10px 0;
  }}
  #rag-chatbot-header {{
    margin-bottom: 10px;
    padding-bottom: 10px;
    border-bottom: 1px solid #eee;
  }}
  #rag-chatbot-messages {{
    height: 300px;
    overflow-y: auto;
    margin-bottom: 10px;
    border: 1px solid #eee;
    padding: 8px;
    background-color: #f9f9f9;
  }}
  .user-message {{
    text-align: right;
    color: #1890ff;
    margin-bottom: 8px;
  }}
  .bot-message {{
    text-align: left;
    color: #555;
    margin-bottom: 8px;
  }}
  #rag-chatbot-input-area {{
    display: flex;
  }}
  #rag-query-input {{
    flex-grow: 1;
    padding: 8px;
    border: 1px solid #ccc;
    border-radius: 4px 0 0 4px;
  }}
  #rag-send-btn {{
    padding: 8px 16px;
    background-color: #1890ff;
    color: white;
    border: none;
    border-radius: 0 4px 4px 0;
    cursor: pointer;
  }}
  #rag-send-btn:hover {{
    background-color: #40a9ff;
  }}
`;
document.head.appendChild(style);
</script>
        """

        return {
            "embed_code": embed_code,
            "book_title": book.title,
            "book_author": book.author,
            "book_id": book_id
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error generating embed code: {str(e)}"
        )
    finally:
        db.close()


@router.post("/run-pipeline", tags=["Embedding Pipeline"])
async def run_embedding_pipeline(current_user: str = Depends(get_current_user)):
    """
    Run the Docusaurus embedding pipeline to store data in Qdrant.
    This endpoint triggers the pipeline that fetches documents, chunks them,
    generates embeddings, and stores them in the Qdrant vector database.
    """
    try:
        # Initialize and run the embedding pipeline
        pipeline = DocusaurusEmbeddingPipeline()
        pipeline.run_pipeline()

        return {
            "message": "Embedding pipeline completed successfully",
            "status": "completed"
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error running embedding pipeline: {str(e)}"
        )