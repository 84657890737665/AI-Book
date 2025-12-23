import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ContextAwareChatbot from '../../components/Chatbot/ContextAwareChatbot';

const ChatbotInjector: React.FC = () => {
  const location = useLocation();

  // Determine if we're on a docs page
  const isDocsPage = location.pathname.startsWith('/docs/');

  // Extract a short page context based on the URL
  const pageContext = isDocsPage
    ? {
        url: location.pathname,
        title: document.title, // Get title from document
      }
    : null;

  return isDocsPage ? (
    <ContextAwareChatbot
      pageContext={JSON.stringify(pageContext)}
    />
  ) : null;
};

export default ChatbotInjector;