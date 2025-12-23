import React, { useState, useEffect, useRef } from 'react';
import styles from './Chatbot.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{id: number, text: string, sender: 'user' | 'bot'}[]>([
    { id: 1, text: 'Hello! I\'m Tan, your AI Assistant for Physical AI & Humanoid Robotics. How can I help you today?', sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selectedTextObj = window.getSelection();
      if (selectedTextObj && selectedTextObj.toString().trim() !== '') {
        setSelectedText(selectedTextObj.toString().trim());
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSend = async (customQuery?: string) => {
    const query = customQuery || inputValue;
    if (query.trim() === '' || isLoading) return;

    // Add user message
    const newUserMessage = {
      id: messages.length + 1,
      text: customQuery ? `About "${selectedText}": ${query}` : query,
      sender: 'user' as const
    };

    setMessages(prev => [...prev, newUserMessage]);

    // Only clear the input if not sending a custom query (like from selected text)
    if (!customQuery) {
      setInputValue('');
    }

    setIsLoading(true);

    try {
      // Call the backend API to get the response
      const response = await fetch('https://tan-ee320-rag-chatbot.hf.space', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          mode: 'FULL_BOOK', // Default mode
          selected_text: selectedText // Send the selected text context
        })
      });

      if (!response.ok) {
        throw new Error(`Backend API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response
      const botResponse = {
        id: messages.length + 2,
        text: data.response,
        sender: 'bot' as const
      };

      setMessages(prev => [...prev, botResponse]);
      // Clear selected text after successful submission
      if (selectedText) {
        setSelectedText(null);
      }
    } catch (error) {
      console.error('Error calling backend API:', error);

      // Add error message
      const errorMessage = {
        id: messages.length + 2,
        text: 'Sorry, I encountered an error while processing your request. Please try again.',
        sender: 'bot' as const
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault(); // Prevent new line in input
      handleSend();
    }
  };

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <>
      {/* Quick Ask Button - appears when text is selected */}
      {selectedText && (
        <div
          className={styles.quickAskButton}
          onClick={() => {
            // Prepopulate input with selected text and prompt user
            setInputValue(`Ask about: "${selectedText}"`);
            setIsOpen(true); // Open the chatbot
          }}
        >
          💬 Ask AI Assistant
        </div>
      )}

      {/* Floating Chatbot Button */}
      <div className={styles.chatbotButton} onClick={toggleChat}>
        <div className={styles.chatbotIcon}>
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            className={styles.icon}
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </div>
      </div>

      {/* Chatbot Panel */}
      {isOpen && (
        <div className={styles.chatbotPanel}>
          <div className={styles.chatHeader}>
            <div className={styles.chatTitle}>AI Assistant</div>
            <button className={styles.closeButton} onClick={toggleChat}>
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="24"
                height="24"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.sender]}`}
              >
                {message.text}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.chatInputContainer}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about Physical AI or Humanoid Robotics..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button onClick={handleSend} className={styles.sendButton} disabled={isLoading}>
              {isLoading ? (
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="20"
                  height="20"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className={styles.spinner}
                >
                  <path d="M21 12a9 9 0 1 1-6.219-8.56" />
                </svg>
              ) : (
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="20"
                  height="20"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              )}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;