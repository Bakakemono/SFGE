/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SFGE_INPUT_H
#define SFGE_INPUT_H

#include <engine/engine.h>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>

namespace sfge
{
class KeyboardManager;
class MouseManager;
/**
* \brief Handles Input like the Keyboard, the Joystick or the Mouse
*/
class InputManager : public Module
{
public:
	using Module::Module;

	InputManager(Engine& engine, bool enable = true);
	/**
	 * \brief Initialize the Input Manager
	 */
	void Init() override;
	/**
	 * \brief Update called each frame to report input status
	 * \param dt Delta time since last frame
	 */
	void Update(sf::Time dt) override;
	void Destroy() override;

	std::shared_ptr<InputManager> GetInputManager();
	std::shared_ptr<KeyboardManager> GetKeyboardManager();
	std::shared_ptr<MouseManager> GetMouseManager();

private:

protected:
	std::shared_ptr<InputManager> m_InputManager = nullptr;
	std::shared_ptr<KeyboardManager> m_KeyboardManager = nullptr;
	std::shared_ptr<MouseManager> m_MouseManager = nullptr;
};

struct KeyPressedStatus { bool previousKeyPressed; bool keyPressed; };

class KeyboardManager
{
public:
	void Update(sf::Time dt);
	bool IsKeyHeld(sf::Keyboard::Key key);
	bool IsKeyDown(sf::Keyboard::Key key);
	bool IsKeyUp(sf::Keyboard::Key key);

protected:
	

private:
	KeyPressedStatus keyPressedStatusArray[sf::Keyboard::Key::KeyCount] ={};
};

class MouseManager
{
public:
	sf::Vector2i localPosition(sf::Window& window);
private:

};

}
#endif
