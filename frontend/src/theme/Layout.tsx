import React from 'react';
import Layout from '@theme-original/Layout';
import ChatBot from '../components/ChatBot';

export default function LayoutWithChatBot(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatBot />
    </>
  );
}