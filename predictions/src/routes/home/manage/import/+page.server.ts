import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, query, where, getDocs, doc, getDoc } from 'firebase/firestore';
import type { DocumentData } from 'firebase/firestore';

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	//Query Events
	const eventsSnapshot = await getDocs(collection(db, 'events'));

	let events: DocumentData[] = [];
	eventsSnapshot.forEach((doc) => {
		events.push(doc.data());
	});

	return {
		events: events
	};
}
