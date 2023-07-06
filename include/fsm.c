#include <stdlib.h>
#include "fsm.h"

fsm_t*
fsm_new (fsm_trans_t* tt)
{
  fsm_t* this_ = (fsm_t*) malloc (sizeof (fsm_t));
  fsm_init (this_, tt);
  return this_;
}

void fsm_init (fsm_t* this_, fsm_trans_t* tt)
{
  this_->tt = tt;
}

void
fsm_fire (fsm_t* this_)
{
  fsm_trans_t* t;
  for (t = this_->tt; t->orig_state >= 0; ++t) {
    if ((this_->current_state == t->orig_state) && t->in(this_)) {
      this_->current_state = t->dest_state;
      if (t->out)
        t->out(this_);
      break;
    }
  }
}

