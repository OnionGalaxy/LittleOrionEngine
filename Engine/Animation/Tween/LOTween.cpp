#include "LOTween.h"
#include "TweenSequence.h"
#include <vector>

TweenSequence* LOTween::CreateSequence()
{
	TweenSequence* sequence = new TweenSequence();
	sequence->state = TweenSequence::TweenSequenceState::DISABLED;
	sequences.emplace_back(sequence);

	return sequence;
}

void LOTween::Update(float dt)
{
	if (sequences.size() <= 0) return;

	int pos = 0;
	for (std::vector<TweenSequence*>::reverse_iterator it = sequences.rbegin(); it != sequences.rend(); ++it)
	{
		TweenSequence* sequence = (*it);

		if (sequence->state == TweenSequence::TweenSequenceState::STOPPED) continue;

		sequence->Update(dt);
	}
}

void LOTween::Reset()
{
	for (std::vector<TweenSequence*>::reverse_iterator it = sequences.rbegin(); it != sequences.rend(); ++it)
	{
		TweenSequence* sequence = (*it);
		delete(sequence);
	}

	sequences.clear();
}
