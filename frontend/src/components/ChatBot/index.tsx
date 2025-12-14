import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: "Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. How can I help you today?",
      role: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    // Simulate API call delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    // Generate a mock response based on the input
    const responses = [
      `Regarding "${inputValue}", this is a simulated response. In a real implementation, this would connect to an AI service to provide accurate information about Physical AI & Humanoid Robotics.`,
      `Thank you for your question about "${inputValue}". This would normally be processed by an AI model trained on the Physical AI & Humanoid Robotics textbook content.`,
      `I understand you're asking about "${inputValue}". In the actual implementation, our AI assistant would provide a detailed answer based on the textbook material.`,
      `Great question about "${inputValue}"! In a production system, I would analyze the textbook content and provide you with relevant information.`
    ];

    const randomResponse = responses[Math.floor(Math.random() * responses.length)];

    const assistantMessage: Message = {
      id: Date.now().toString(),
      content: randomResponse,
      role: 'assistant',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, assistantMessage]);
    setIsLoading(false);
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  return (
    <>
      {/* Chatbot icon button */}
      <button
        className={styles.chatbotIcon}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        <img
          src="/img/chatbot_icon.svg"
          alt="Chatbot"
          className={styles.chatbotIconImg}
        />
      </button>

      {/* Chatbot modal */}
      {isOpen && (
        <div className={styles.chatbotModal}>
          <div className={styles.chatbotHeader}>
            <h3>AI Assistant</h3>
            <button className={styles.closeButton} onClick={closeChat}>
              ×
            </button>
          </div>
          <div className={styles.chatbotContent}>
            <div className={styles.chatMessages}>
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${styles[message.role]}`}
                >
                  <div className={styles.messageContent}>
                    {message.content}
                  </div>
                  <div className={styles.messageTime}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))}
              {isLoading && (
                <div className={`${styles.message} ${styles.assistant}`}>
                  <div className={styles.messageContent}>
                    <div className={styles.typingIndicator}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className={styles.chatInputArea}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about Physical AI & Humanoid Robotics..."
                disabled={isLoading}
                className={styles.chatInput}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                className={styles.sendButton}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatBot;