
#ifndef H_PROG_BAR_ITFC
#define H_PROG_BAR_ITFC
#pragma once

namespace ggeomlib
{
	/**
	* @class IProgressBarInterface
	* @brief Abstract interface for reporting progress and handling cancellation requests.
	*
	* This interface defines a contract for progress reporting in long-running operations.
	* It supports updating progress (0–100), incrementing step-based progress, setting the
	* current processing phase name, and managing cancellation state.
	*
	* Implementations may be used by GUI progress bars, CLI tools, or background task systems.
	*/
	class IProgressBarInterface
	{
	public:

		/**
		* @brief Initializes the progress bar (for example showing a window or setting parameters
		*		 to default values, this depends on the specific implementation and is completely
		*		 left to the user)
		*/
		virtual void Initialize() = 0;

		/**
		* @brief Cleans everything when the progress bar is no longer needed (implementation is completely
		*		 upt to the user)
		*/
		virtual void Terminate() = 0;

		/**
		* @brief Updates the current progress status.
		* @param f_progress Progress value in the range [0, 100].
		* @param f_phase_name Optional textual description of the current processing phase.
		*/
		virtual void SetProgress(int f_progress) = 0;

		/**
		* @brief Increments progress by a single step.
		*
		* Useful when progress is based on discrete operations and the total number of steps
		* is managed internally by the implementation.
		*/
		virtual void IncrementOneProgressStep() = 0;

		/**
		* @brief Sets the name of the current processing phase.
		* @param f_phase_name Null-terminated string describing the current phase.
		*/
		virtual void SetPahseName(const char* f_phase_name) = 0;

		/**
		* @brief Sets the weight of the current phase in the overall progress computation.
		*
		* The weight determines how much the current phase contributes to the total progress.
		* For example, a weight of 10 means the phase contributes 10% of the total work.
		*
		* @param f_weight Weight of the current phase (typically expressed as a percentage or relative unit).
		*/
		virtual void SetPhaseWeight(const int f_weigth) = 0;

		/**
		* @brief Queries whether cancellation has been requested.
		* @return True if cancellation was requested, false otherwise.
		*/
		virtual bool GetCancellationRequested() = 0;

		/**
		* @brief Sets the cancellation state.
		* @param f_cancellation_requested True to request cancellation, false otherwise.
		*/
		virtual void SetCancellationRequested(const bool f_cancellation_requested) = 0;

		/**
		* @brief Virtual destructor.
		*/
		virtual ~IProgressBarInterface() = default;

	};
}

#endif