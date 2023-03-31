import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, query, where, getDocs, orderBy } from 'firebase/firestore';
import type { DocumentData } from 'firebase/firestore';

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	//Event ID
	const slug = params.slug;

	//Query Documents
	const q = query(
		collection(db, 'matches'),
		where('event', '==', slug),
		where('active', '==', true),
		orderBy('match_number')
	);
	const querySnapshot = await getDocs(q);

	let matches: DocumentData[] = [];
	let ids: string[] = [];
	querySnapshot.forEach((doc) => {
		matches.push(doc.data());
		ids.push(doc.id);
	});

	if (matches) {
		return {
			matches: matches,
			match_id: ids
		};
	}

	throw error(404, 'Not found');
}
