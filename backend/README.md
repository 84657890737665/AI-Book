# Integrated RAG Chatbot

This project implements an integrated Retrieval-Augmented Generation (RAG) chatbot that allows users to interact with book content using natural language queries. The system is built with Python, FastAPI, Cohere API, Neon Postgres, and Qdrant vector store.

## Features

- **Natural Language Queries**: Ask questions about book content in plain English
- **Selected Text Mode**: Focus queries on specific text selections within the book
- **Citation References**: Get proper citations linking responses to source passages
- **Embeddable Widget**: Integrate the chatbot directly into PDF/EPUB books
- **Secure Authentication**: JWT-based authentication for user sessions

## Tech Stack

- **Backend**: Python 3.11, FastAPI
- **AI/ML**: Cohere API (Command R+ for generation, Embed for vectors)
- **Database**: Neon Serverless Postgres (with pgvector extension)
- **Vector Store**: Qdrant Cloud Free Tier
- **Authentication**: JWT-based
- **Frontend**: Embeddable HTML/JS widget

## Setup

### Prerequisites

- Python 3.11+
- Pip package manager
- Git
- Cohere API key
- Access to Neon Serverless Postgres
- Access to Qdrant Cloud Free Tier

### Local Development

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Copy the environment file and update with your credentials:
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   ```

5. Initialize the database:
   ```bash
   # Run database migrations
   alembic upgrade head
   ```

6. Start the development server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

## Usage

### 1. Upload a book

To use the RAG chatbot, first upload a book in PDF or EPUB format:

```bash
curl -X POST "http://localhost:8000/books" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/your/book.pdf" \
  -F "title=Book Title" \
  -F "author=Author Name"
```

The book will be processed automatically, with chunks stored in Qdrant for retrieval.

### 2. Query the book content

Once processing is complete, you can ask questions about the book:

```bash
curl -X POST "http://localhost:8000/query/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main concepts covered in this book?",
    "mode": "full_book"
  }'
```

To query only specific selected text:

```bash
curl -X POST "http://localhost:8000/query/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept in simpler terms",
    "mode": "selected_text",
    "selected_text": "The selected text that you want to focus on for this query..."
  }'
```

### 3. Embed in your book

To get the embeddable code for a book:

```bash
curl -X GET "http://localhost:8000/embed/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

## API Endpoints

- `POST /auth/login` - Authenticate and get JWT token
- `POST /books` - Upload and process a new book
- `GET /books/{bookId}` - Get information about a book
- `POST /query/{bookId}` - Query book content using RAG
- `GET /embed/{bookId}` - Get embeddable chatbot code

## Security

- All endpoints except `/auth/login` require a valid JWT token
- Communication with Cohere API, Neon, and Qdrant is encrypted
- Sensitive credentials are stored in environment variables

## License

This project is licensed under the MIT License.