// chatbot-loader.js
// This script will dynamically load and render the chatbot component after the page loads

document.addEventListener('DOMContentLoaded', function() {
  // Create the chatbot container
  const chatbotContainer = document.createElement('div');
  chatbotContainer.id = 'chatbot-container';
  document.body.appendChild(chatbotContainer);

  // Since we can't directly render React components from a plain JS file,
  // we'll create a simple vanilla JS implementation for now
  // In a real scenario, this would be replaced with proper React mounting

  // Create the chatbot button
  const chatbotButton = document.createElement('div');
  chatbotButton.className = 'chatbot-container';
  chatbotButton.innerHTML = `
    <button class="chatbot-toggle-button" aria-label="Open chatbot">
      <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="24" height="24" fill="currentColor">
        <path d="M20 2H4c-1.1 0-1.99.9-1.99 2L2 22l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-2 12H6v-.01C6 10.03 7.53 9 9.49 9h.02c1.82 0 3.31 1.09 3.82 2.74l.12.38.12-.38c.51-1.65 2-2.74 3.82-2.74h.02c.17 0 .33.01.49.03V14z"/>
      </svg>
    </button>
  `;

  document.body.appendChild(chatbotButton);

  // Add basic styles dynamically
  const style = document.createElement('style');
  style.textContent = `
    .chatbot-container {
      position: fixed;
      bottom: 20px;
      right: 20px;
      z-index: 1000;
    }

    .chatbot-toggle-button {
      width: 60px;
      height: 60px;
      border-radius: 50%;
      background-color: #007acc;
      border: none;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
      transition: transform 0.2s, box-shadow 0.2s;
      color: inherit;
    }

    .chatbot-toggle-button:hover {
      transform: scale(1.05);
      box-shadow: 0 6px 12px rgba(0, 0, 0, 0.3);
    }
  `;
  document.head.appendChild(style);

  // Add click handler to the button
  const button = chatbotButton.querySelector('.chatbot-toggle-button');
  button.addEventListener('click', function() {
    alert('Chatbot would open here. In a full implementation, this would open the React chatbot component.');
  });
});