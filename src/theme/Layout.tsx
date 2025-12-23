import React, { useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function Layout(props) {
  const [showChatbot, setShowChatbot] = React.useState(false);

  useEffect(() => {
    // Show chatbot only on docs pages (pages that start with /docs/)
    if (window && window.location && window.location.pathname.startsWith('/docs/')) {
      setShowChatbot(true);
    }
  }, []);

  return (
    <>
      <OriginalLayout {...props} />
      {showChatbot && <Chatbot />}
    </>
  );
}