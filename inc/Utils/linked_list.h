
#ifndef LINKEDLIST_H
#define LINKEDLIST_H

// linkedlist.h
//
// A linked list structure
//
// @author: sahmadi
// @updated: 06/06/21
//

#include <cstring>
#include <stdint.h>

template <class T>
class linkedlist
{
public:
	linkedlist(T element = 0) : head_(element), tail_(element)
	{
		element ? size_ = 1 : size_ = 0;
	}

	// ~linkedlist()
	// { }

	inline void
	push_front(T element)
	{
		element->set_next(head_);
		head_ = element;
		size_++;
		
		// if (!tail_)
		// tail_ = head_;
		
	}
	
	inline void
	push_back(T element)
	{
		(element)->set_next(0);
		if (!head_)
		{
			head_ = element;
			tail_ = element;
			// tail_prev_ = 0;
		}
		else
		{
			// tail_prev_ = tail_;
			(tail_)->set_next(element);
			tail_ = element;
		}
		size_++;
	}

	inline void
	set_head(T element)
	{
		head_ = element;
	}

	inline void
	set_tail(T element)
	{
		tail_ = element;
	}

	inline void
	set_prev_back(T element)
	{
		tail_prev_ = element;
	}

	inline void
	set_size(unsigned int size)
	{
		size_ = size;
	}

	inline T
	front()
	{
		return head_;
	}

	inline T
	back()
	{
		return tail_;
	}

	inline T
	prev_back()
	{
		return tail_prev_;
	}

	inline unsigned int
	size()
	{
		return size_;
	}

	inline void
	pop_front()
	{
		// we never pop from empty list!
		// if(head_)
		// {
			// if (size_ <= 1)
			// {
			// 	clear();
			// }
			// else
			{
          head_ = (T) head_->get_next();
				size_--;
			}
			
		// }
	}

	// THIS must be followed by a push_back()
	// Otherwise, tail_prev_ does not change.
	inline void
	pop_back()
	{
		// we never pop from empty list!
		// if (size_ <= 1)
		// {
		// 	clear();
		// }
		// else
		{
			tail_ = tail_prev_;
			size_--;
		}
	}

	inline void
	clear()
	{
		head_ = 0;
		tail_ = 0;
		size_ = 0;
		tail_prev_ = 0;
	}

	size_t
	mem() { return sizeof(T) * size_ * 2; }

	linkedlist<T> &
	operator=(linkedlist<T> &other)
	{
		this->head_ = other.head_;
		this->tail_ = other.tail_;
		this->tail_prev_ = other.tail_prev_;
		this->size_ = other.size_;
		// other.size_ = 0;
		return *this;
	}

private:
	T head_= 0;
	T tail_ = 0;
	T tail_prev_ = 0;
	unsigned int size_ = 0;
};

#endif
