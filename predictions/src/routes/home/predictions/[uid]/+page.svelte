<script>
	export let data;
	const { predictions, matches } = data;
</script>

<div class="grid h-screen place-items-center content-start md:content-center">
	<div class="md:card md:w-200 md:bg-base-300 md:shadow-xl">
		<div class="card-body">
			<h2 class="card-title pb-7">Your Predictions</h2>
			<div class="flex space-x-40">
				<h2 class="pl-10">Match Info</h2>
				<h2>You</h2>
			</div>
			{#each matches as match, index}
				<a class="relative" href="">
					<div class="flex items-center space-x-2">
						<div class="btn-group">
							<button class="btn btn-square btn-primary">{match.match_number}</button>
							<button class="btn btn-square btn-error">{match.red_team[0]}</button>
							<button class="btn btn-square btn-error">{match.red_team[1]}</button>
							<button class="btn btn-square btn-error">{match.red_team[2]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[0]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[1]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[2]}</button>
						</div>
						{#if predictions[index].prediction == 'Blue Alliance'}
							<button class="btn btn-square btn-accent">
								{predictions[index].amount}
							</button>
						{:else if predictions[index].prediction == 'Red Alliance'}
							<button class="btn btn-square btn-error">
								{predictions[index].amount}
							</button>
						{/if}
					</div>
				</a>
			{/each}
		</div>
	</div>
</div>
