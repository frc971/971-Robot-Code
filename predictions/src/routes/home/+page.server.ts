import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, getDocs, doc, getDoc } from 'firebase/firestore';
import type { DocumentData } from 'firebase/firestore';

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	//Query Events
	const querySnapshot = await getDocs(collection(db, 'events'));

	let events: DocumentData[] = [];
	querySnapshot.forEach((doc) => {
		events.push(doc.data());
	});

	if (events) {
		return {
			events: events
		};
	}

	throw error(404, 'Not found');
}
