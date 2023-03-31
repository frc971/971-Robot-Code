<script>
	import { doc, setDoc } from 'firebase/firestore';
	import { db } from '$lib/firestore';
	import { goto } from '$app/navigation';

	export let data;
	const { events, matches } = data;

	let eventSelect = '';
	let matchNumberInput = '';
	let blue_team = '';
	let red_team = '';

	async function handleCreate() {
		const docID = eventSelect + '-' + matchNumberInput;
		const blue_team_array = blue_team.split(',');
		const red_team_array = red_team.split(',');
		console.log(blue_team_array);
		console.log(red_team_array);
		await setDoc(doc(db, 'matches', docID), {
			active: true,
			event: eventSelect,
			match_finished: false,
			match_number: matchNumberInput,
			match_result: 'Red Alliance',
			blue_team: blue_team_array,
			red_team: red_team_array
		});

		goto('/home/manage');
	}
</script>

<div class="grid h-screen place-items-center">
	<div class="card w-200 bg-base-300 shadow-xl">
		<div class="card-body">
			<div class="flex space-x-2">
				<button on:click={() => goto('/home/manage')} class="btn btn-square btn-xs btn-primary"
					>&#8592</button
				>
				<h2 class="card-title pb-7">Create Match</h2>
			</div>
			<div class="flex space-x-2">
				<div>
					<h4>Select Event</h4>
					<select bind:value={eventSelect} class="select select-bordered w-full max-w-xs">
						<option disabled selected>Event</option>
						{#each events as event}
							<option>{event.id}</option>
						{/each}
					</select>
				</div>
				<div>
					<h4 class="">Match Number</h4>
					<div class="">
						<input
							bind:value={matchNumberInput}
							type="text"
							placeholder="Type here"
							class="input input-bordered input-primary w-full max-w-xs"
						/>
					</div>
				</div>
			</div>
			<div>
				<div class="pt-2">
					<input
						bind:value={blue_team}
						type="text"
						placeholder="Blue Team | Ex. 971,972,973"
						class="input input-bordered input-accent w-full max-w-xs"
					/>
				</div>
			</div>
			<div>
				<div class="pt-2">
					<input
						bind:value={red_team}
						type="text"
						placeholder="Red Team | Ex. 971,972,973"
						class="input input-bordered input-error w-full max-w-xs"
					/>
				</div>
			</div>
			<button on:click={handleCreate} class="btn btn-primary">Create</button>
		</div>
	</div>
</div>
