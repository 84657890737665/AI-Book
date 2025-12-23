import React from 'react';
import OriginalDocItem from '@theme-original/DocItem';
// Not adding chatbot here to prevent excessive memory consumption during build

export default function DocItem(props) {
  return (
    <OriginalDocItem {...props} />
  );
}