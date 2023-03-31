<script>
	export let data;
	const { events } = data;
</script>

<div class="grid h-screen place-items-center">
	<div class="md:card md:w-80 md:bg-base-300 md:shadow-xl">
		<div class="card-body">
			<h2 class="card-title pb-7">Active Events</h2>
			{#each events as event}
				<a class="relative" href="/home/events/{event.id}">
					<button class="btn btn-wide btn-primary">{event.title}</button>
				</a>
			{/each}
		</div>
	</div>
</div>
