# Data Model: Integrated RAG Chatbot

**Feature**: Integrated RAG Chatbot Development
**Date**: 2025-12-13
**Status**: Design

## Entities

### BookContent
**Description**: Represents the text content of a published book that serves as the knowledge base for the RAG system

**Fields**:
- `id` (UUID, Primary Key): Unique identifier for the book
- `title` (String, NOT NULL): The title of the book
- `author` (String, NOT NULL): The author of the book
- `isbn` (String, NULLABLE): ISBN identifier for the book
- `content_type` (Enum: PDF, EPUB, TEXT, OTHER): Format of the original content
- `content_path` (String, NOT NULL): File path or URL to the original book content
- `total_pages` (Integer, NULLABLE): Number of pages in the book
- `language` (String, NOT NULL): Language of the book content
- `processing_status` (Enum: PENDING, PROCESSING, COMPLETED, FAILED): Current status of content processing
- `created_at` (DateTime, NOT NULL): Timestamp when the book was added
- `updated_at` (DateTime, NOT NULL): Timestamp when the book was last updated

**Relationships**:
- One-to-Many with ProcessedContent (via book_id)

**Validation Rules**:
- Title must be between 1 and 500 characters
- Author must be between 1 and 200 characters
- Content path must be a valid path/URL
- Processing status must be one of the defined enums

### ProcessedContent
**Description**: Represents the automatically chunked and embedded text sections ready for RAG retrieval

**Fields**:
- `id` (UUID, Primary Key): Unique identifier for the processed content
- `book_id` (UUID, Foreign Key to BookContent.id): Reference to the original book
- `chunk_id` (String, NOT NULL): Unique identifier for the chunk in the vector store
- `chunk_text` (Text, NOT NULL): The actual text content of the chunk
- `chunk_metadata` (JSONB, NOT NULL): Metadata about the chunk (page number, section, etc.)
- `embedding_id` (String, NOT NULL): ID of the embedding in the vector store
- `created_at` (DateTime, NOT NULL): Timestamp when the chunk was created

**Relationships**:
- Many-to-One with BookContent (via book_id)

**Validation Rules**:
- Chunk text must be between 10 and 4096 characters
- Chunk metadata must be valid JSON
- Book ID must reference an existing BookContent record

### UserQuery
**Description**: Represents the question or prompt submitted by the user for which the system generates a response

**Fields**:
- `id` (UUID, Primary Key): Unique identifier for the query
- `user_id` (UUID, NOT NULL): Identifier for the authenticated user
- `book_id` (UUID, Foreign Key to BookContent.id): Reference to the book being queried
- `query_text` (Text, NOT NULL): The question or prompt submitted by the user
- `query_mode` (Enum: FULL_BOOK, SELECTED_TEXT): Whether querying full book or selected text
- `selected_text` (Text, NULLABLE): If in SELECTED_TEXT mode, the specific text provided
- `retrieved_context` (JSONB, NOT NULL): Context retrieved from the RAG system
- `response_text` (Text, NOT NULL): The AI-generated response to the query
- `citation_references` (JSONB, NOT NULL): Links to specific source passages
- `query_latency` (Float, NULLABLE): Time taken to generate the response in seconds
- `created_at` (DateTime, NOT NULL): Timestamp when the query was submitted

**Relationships**:
- Many-to-One with BookContent (via book_id)

**Validation Rules**:
- Query text must be between 1 and 2000 characters
- Selected text must be provided when query_mode is SELECTED_TEXT
- Query mode must be one of the defined enums
- Book ID must reference an existing BookContent record

### MultimediaContent
**Description**: Images, tables, diagrams, and other non-text elements in the book with associated text descriptions

**Fields**:
- `id` (UUID, Primary Key): Unique identifier for the multimedia content
- `book_id` (UUID, Foreign Key to BookContent.id): Reference to the associated book
- `content_type` (Enum: IMAGE, TABLE, DIAGRAM, AUDIO, VIDEO): Type of multimedia content
- `content_path` (String, NOT NULL): Path to the multimedia file
- `text_description` (Text, NOT NULL): Text description of multimedia content
- `alt_text` (Text, NULLABLE): Alternative text for accessibility
- `metadata` (JSONB, NULLABLE): Additional metadata about the multimedia
- `created_at` (DateTime, NOT NULL): Timestamp when the content was added

**Relationships**:
- Many-to-One with BookContent (via book_id)

**Validation Rules**:
- Content path must be a valid path/URL
- Content type must be one of the defined enums
- Book ID must reference an existing BookContent record

### UserSession
**Description**: Represents an authenticated user session for the RAG chatbot

**Fields**:
- `id` (UUID, Primary Key): Unique identifier for the session
- `user_id` (String, NOT NULL): Unique identifier for the user
- `session_token` (String, NOT NULL, UNIQUE): JWT token for the session
- `created_at` (DateTime, NOT NULL): Timestamp when the session was created
- `expires_at` (DateTime, NOT NULL): Timestamp when the session expires
- `active` (Boolean, NOT NULL, DEFAULT: true): Whether the session is still active

**Relationships**:
- One-to-Many with UserQuery (via user_id)

**Validation Rules**:
- Session token must be a valid JWT
- Expires_at must be after created_at
- Active must be a boolean value

## State Transitions

### BookContent Processing
- PENDING → PROCESSING: When processing begins
- PROCESSING → COMPLETED: When all chunks are successfully embedded
- PROCESSING → FAILED: When an error occurs during processing
- FAILED → PENDING: When user retries the processing

## Indexes

### BookContent
- Index on `title` for fast searching
- Index on `processing_status` for filtering by status
- Composite index on `author` and `title` for combined searches

### ProcessedContent
- Index on `book_id` for quick book-based retrieval
- Index on `chunk_id` for vector store lookups
- GIN index on `chunk_metadata` for metadata-based searches

### UserQuery
- Index on `user_id` for user-specific queries
- Index on `book_id` for book-specific queries
- Index on `created_at` for chronological ordering