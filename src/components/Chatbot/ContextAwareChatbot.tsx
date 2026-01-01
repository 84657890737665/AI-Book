import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Chatbot.module.css';

interface Message {
  id: number;
  text: string;
  sender: 'user' | 'bot';
}

interface ContextAwareChatbotProps {
  pageContext?: string; // Pass the current page/module context
}

const ContextAwareChatbot: React.FC<ContextAwareChatbotProps> = ({ pageContext = '' }) => {
  const { siteConfig } = useDocusaurusContext();
  const chatbotApiUrl = (siteConfig.customFields?.chatbotApiUrl as string) || 'http://localhost:8000/query';

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    { id: 1, text: 'Hello! I\'m Tan, your AI Assistant for Physical AI & Humanoid Robotics. How can I help you with this page?', sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Function to handle text selection
  const handleTextSelection = () => {
    const selectedTextObj = window.getSelection();
    if (selectedTextObj && selectedTextObj.toString().trim() !== '') {
      setSelectedText(selectedTextObj.toString().trim());
    }
  };

  // Set up text selection event listener
  useEffect(() => {
    const handleMouseUp = () => {
      handleTextSelection();
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleSend = async () => {
    if (inputValue.trim() === '' || isLoading) return;

    // Use selected text in the query if available
    const queryText = selectedText ? `${inputValue} (about: "${selectedText}")` : inputValue;

    // Add user message
    const newUserMessage: Message = {
      id: messages.length + 1,
      text: inputValue, // Show the original input to user
      sender: 'user'
    };

    setMessages(prev => [...prev, newUserMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API to get the response
      const response = await fetch(chatbotApiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: queryText, // Include selected text in the query
          mode: selectedText ? 'SELECTED_TEXT' : 'FULL_BOOK',
          selected_text: selectedText,
          context: pageContext || null // Add page context if available
        })
      });

      if (!response.ok) {
        throw new Error(`Backend API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response
      const botResponse: Message = {
        id: messages.length + 2,
        text: data.response || "I'm sorry, I couldn't generate a response. Please try asking in a different way.",
        sender: 'bot'
      };

      setMessages(prev => [...prev, botResponse]);
      setSelectedText(null); // Clear selected text after sending
    } catch (error) {
      console.error('Error calling backend API:', error);

      // Add error message
      const errorMessage: Message = {
        id: messages.length + 2,
        text: 'Sorry, I encountered an error while processing your request. Please try again.',
        sender: 'bot'
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
      {/* Floating Chatbot Button */}
      {selectedText && (
        <div
          className={styles.quickAskButton}
          onClick={() => {
            setInputValue(`Ask about: "${selectedText}"`);
            setIsOpen(true);
            setSelectedText(null);
          }}
        >
          <span>💬 Ask about this</span>
        </div>
      )}

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

export default ContextAwareChatbot;