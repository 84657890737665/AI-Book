import { LoadContext, Plugin } from '@docusaurus/types';
import path from 'path';

export default function createChatbotPlugin(
  context: LoadContext
): Plugin<void> {
  return {
    name: 'docusaurus-context-aware-chatbot',

    getThemePath() {
      return path.resolve(__dirname, './theme');
    },
  };
}