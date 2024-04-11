#pragma once
// A bucket-based priority queue for search_labels. 
// WITHOUT tie breaking using linked lists inside buckets
// First implementation pqueue_label_bucket: Fixed range buckets (needs f_max-f_min)
// Second implementation pqueue_label_bucket_cyclic: Cyclic buckets (needs bucket_size)
//
// 
//
// @author: sahmadi
// @created: 04/06/2021
// @updated: 28/01/2022
//

#include <cassert>
#include <iostream>
#include "Definitions.h"
#include "linked_list.h"
#include<boost/heap/priority_queue.hpp>
#include <sys/_types/_size_t.h>

template <class LABEL>
class pqueue_label_bucket
{
    
	public:
    
        pqueue_label_bucket(size_t size=1, cost_t fmin=0, cost_t fmax=0)
            : max_cursor_(0), queuesize_(0), cursor_(0), f_min(fmin)
        {
            elts_ = new linkedlist<LABEL*>[1]();
            bucket_width_ = 1;
            initsize_ = (fmax - fmin)/bucket_width_ + 1;
            resize(initsize_);
        }

        ~pqueue_label_bucket()
        {
            delete [] elts_;
        }

        inline void
        range_update(cost_t fmin, cost_t fmax)
        {
            f_min = fmin;
            resize(fmax-fmin+1);
        }

        void
        clear()
        {
            queuesize_ = 0;
            max_cursor_ = initsize_;
            delete [] elts_;
            elts_ = 0;
            cursor_ = 0;
        }

		// add a new element to the pqueue
        void 
        push (LABEL* lb)
        {
            size_t index = (lb->f.at(0) - f_min);
            if (index > max_cursor_){
                size_t new_size = max_cursor_ * 2;
                while(index > new_size){new_size = new_size * 2;}
                resize(new_size);
            }
            this->elts_[index].push_front(lb);
            
            queuesize_++;
            return;
        }

		LABEL*
        top()
        {
            if (queuesize_)
            {
                LABEL* ans = this->peek();
                return ans;
            }
            return nullptr;
        }

	void	
    pop()
    {
        if (queuesize_)
            {
                (this->elts_[cursor_]).pop_front();
                queuesize_--;
            }
    }
		
		inline LABEL*
		peek()
		{
            while(!elts_[cursor_].size())
                {cursor_++;}
                
            return this->elts_[cursor_].front();
        }

        // inline cost_t 
		// peek_f(void)
		// {
        //     LABEL* ans = this->peek();
            
        //     if(ans)
		// 	{
        //         return this->cursor_+this->f_min;
        //     }
        //     // std::cerr<<"Error: Accessing out of Bucket list."<<std::endl;
        //     return COST_MAX;
        // }

        // inline cost_t 
		// current_f(void)
		// {
        //     return this->cursor_+this->f_min;
        // }

		inline unsigned int
		size() const
		{
			return queuesize_;
		}

    void resize(size_t newsize){
        if(newsize - 1 <= max_cursor_){
            return;
        }

        linkedlist<LABEL*>* tmp_ = new linkedlist<LABEL*>[newsize]();

        for(size_t i=0; i <= max_cursor_; i++){
            tmp_[i] = elts_[i];
        }

        delete [] elts_;
        elts_ = tmp_;
        max_cursor_ = newsize-1;
        return;
    }

        inline size_t
        get_cmp()
        {
            return cursor_;
        }

        // void 
        // pull_up(LABEL* lb, LABEL* lb_old)
        // {
        //     lb_old->set_f_pri(0);
        //     this->push(lb);
        //     return;
        
        // }


        void 
        pull_down(LABEL* lb)
        {
            return;
        }
        
        // inline void
        // size_update(cost_t fmax)
        // {
        //     uint new_size = fmax - f_min +1;
        //     while(max_cursor_+1 != new_size)
        //     {
        //         while(this->elts_head_[max_cursor_])
        //             {
        //                 L* tmp = this->elts_head_[max_cursor_];
        //                 this->elts_head_[max_cursor_]=tmp->next_;
        //                 label_pool->save_label(tmp);
        //                 queuesize_--;
        //             } 
        //         // queuesize_ = queuesize_ - counter[max_cursor_];
        //         max_cursor_--;
        //     }
        // }

        size_t
		mem()
		{
			return max_cursor_*sizeof (LABEL*)
				+ sizeof(*this);
		}

		size_t initsize_;
    private:
		size_t max_cursor_;
		unsigned int queuesize_;
		size_t cursor_;
        cost_t f_min;
        size_t bucket_width_;
        linkedlist<LABEL*>* elts_;
    
};

template class pqueue_label_bucket<Node<2>>;


typedef boost::heap::priority_queue<Node<2>*,
    boost::heap::compare<Node<2>::more_than_tr_cost> >  pq_node_2;
