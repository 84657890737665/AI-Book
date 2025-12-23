import React from 'react';
import OriginalDocPage from '@theme-original/DocPage';
// Not adding chatbot here to prevent excessive memory consumption during build

export default function DocPage(props) {
  return (
    <OriginalDocPage {...props} />
  );
}