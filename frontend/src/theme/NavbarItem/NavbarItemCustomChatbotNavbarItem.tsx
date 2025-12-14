import React from 'react';
import NavbarItem from '@theme/NavbarItem';
import { useLocation } from '@docusaurus/router';
import ChatBot from '@site/src/components/ChatBot';

const NavbarItemCustomChatbotNavbarItem = (props) => {
  const { pathname } = useLocation();
  const { mobile } = props;

  // Don't show the chatbot on mobile for now (can be customized)
  if (mobile) {
    return null;
  }

  // Always render the chatbot component regardless of pathname
  return <ChatBot />;
};

export default NavbarItemCustomChatbotNavbarItem;