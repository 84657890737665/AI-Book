import React, { useEffect } from 'react';
import OriginalDocPageLayout from '@theme-original/DocPage/Layout';
import ContextAwareChatbot from '@site/src/components/Chatbot/ContextAwareChatbot';

export default function DocPageLayout(props) {
  const [showChatbot, setShowChatbot] = React.useState(false);

  useEffect(() => {
    // Only show the chatbot on doc pages by checking URL
    if (typeof window !== 'undefined' && window.location.pathname.startsWith('/docs/')) {
      setShowChatbot(true);
    }
  }, []);

  return (
    <>
      <OriginalDocPageLayout {...props} />
      {showChatbot && <ContextAwareChatbot />}
    </>
  );
}