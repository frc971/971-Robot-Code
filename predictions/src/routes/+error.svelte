<script>
	import { uid } from '$lib/store';
	let user_id = $uid;
</script>

<section class="flex items-center h-full">
	<div class="container flex flex-col items-center justify-center px-5 mx-auto my-8">
		<div class="max-w-md text-center">
			<h2 class="mb-8 font-extrabold text-9xl text-primary">
				<span class="sr-only" />404
			</h2>
			<p class="text-2xl font-semibold md:text-3xl pb-5">Uh Oh! Page not found! Error</p>
			<a href="/"><button class="btn btn-primary">Back to homepage</button></a>
		</div>
	</div>
</section>
